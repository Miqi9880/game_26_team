#include <rclcpp/rclcpp.hpp>
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp_components/register_node_macro.hpp"

// awakening auto_aim core headers
#include "tasks/auto_aim/armor_control/very_aimer.hpp"
#include "tasks/auto_aim/armor_detect/armor_detector.hpp"
#include "tasks/auto_aim/armor_track/armor_tracker.hpp"
#include "tasks/auto_aim/auto_aim_fsm.hpp"
#include "tasks/auto_aim/type.hpp"
#include "tasks/base/common.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include <atomic>
#include <cstdio>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

namespace rm_auto_aim
{
namespace aa = awakening::auto_aim;

/// Convert a ROS2 stamp into awakening system_clock TimePoint.
static awakening::TimePoint to_timepoint(const builtin_interfaces::msg::Time & t)
{
  return awakening::TimePoint(
    std::chrono::nanoseconds(static_cast<int64_t>(t.sec) * 1000000000LL + t.nanosec));
}

class AutoAimNode : public rclcpp::Node
{
public:
  explicit AutoAimNode(const rclcpp::NodeOptions & options)
  : Node("auto_aim", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // ---- parameters ----
    const std::string config_path = this->declare_parameter<std::string>(
      "config_path", "/home/nvidia/game_v2/src/serical_device_ros2/config/auto_aim_params.yaml");
    auto_aim_enabled_ = this->declare_parameter<bool>("auto_aim_enable", false);
    allow_fire_ = this->declare_parameter<bool>("allow_fire", false);
    require_mode33_ = this->declare_parameter<bool>("require_mode33", true);
    this->declare_parameter<double>("pitch_zero_offset_deg", 0.987);
    this->declare_parameter<double>("cmd_pitch_max_deg", 15.0);
    this->declare_parameter<double>("cmd_yaw_max_deg", 60.0);
    this->declare_parameter<double>("cmd_slew_deg_per_s", 12.0);
    this->declare_parameter<double>("cmd_yaw_deadband_deg", 0.1);
    this->declare_parameter<double>("cmd_pitch_deadband_deg", 0.1);
    this->declare_parameter<double>("cmd_pitch_delta_max_deg", 3.0);
    this->declare_parameter<double>("cmd_yaw_delta_max_deg", 6.0);
    this->declare_parameter<double>("operator_yaw_offset", 0.0);
    this->declare_parameter<double>("operator_pitch_offset", 0.0);
    this->declare_parameter<double>("vision_delay_ms", 0.0);
    this->declare_parameter<double>("cmd_smooth_alpha", 0.5);
    this->declare_parameter<bool>("aim_center_force", false);
    this->declare_parameter<std::string>("aim_mode", "awakening");
    this->declare_parameter<double>("direct_gain", 1.0);
    this->declare_parameter<double>("direct_pitch_gain", 0.7);
    this->declare_parameter<double>("direct_ki", 0.2);
    this->declare_parameter<double>("direct_fire_scale", 2.5);
    this->declare_parameter<double>("direct_yaw_bias", 0.0);
    this->declare_parameter<double>("direct_pitch_bias", 0.0);
    enemy_color_ = this->declare_parameter<std::string>("enemy_color", "blue");
    bullet_speed_ = this->declare_parameter<double>("bullet_speed", 11.0);

    use_vision_feedback_ = this->declare_parameter<bool>("use_vision_feedback", true);
    YAML::Node cfg = YAML::LoadFile(config_path);
    if (cfg["enemy_color"]) { enemy_color_ = cfg["enemy_color"].as<std::string>(); }
    if (cfg["bullet_speed"]) { bullet_speed_ = cfg["bullet_speed"].as<double>(); }
    if (cfg["use_vision_feedback"]) { use_vision_feedback_ = cfg["use_vision_feedback"].as<bool>(); }
    if (cfg["allow_fire"]) { allow_fire_ = cfg["allow_fire"].as<bool>(); }
    if (cfg["require_mode33"]) { require_mode33_ = cfg["require_mode33"].as<bool>(); }
    if (cfg["camera_offset_m"]) {
      const std::vector<double> v = cfg["camera_offset_m"].as<std::vector<double>>();
      if (v.size() >= 3) { cam_offset_[0] = v[0]; cam_offset_[1] = v[1]; cam_offset_[2] = v[2]; }
    }
    if (cfg["pitch_up_positive"]) { pitch_up_positive_ = cfg["pitch_up_positive"].as<bool>(); }
    if (cfg["yaw_left_positive"]) { yaw_left_positive_ = cfg["yaw_left_positive"].as<bool>(); }
    enemy_ = (enemy_color_ == "red") ? aa::ArmorColor::RED : aa::ArmorColor::BLUE;

    // optional overrides from config file for direct-mode params (declared above)
    {
      auto set_s = [&](const char * k) {
        if (cfg[k]) { this->set_parameter(rclcpp::Parameter(k, cfg[k].as<std::string>())); }
      };
      auto set_d = [&](const char * k) {
        if (cfg[k]) { this->set_parameter(rclcpp::Parameter(k, cfg[k].as<double>())); }
      };
      auto set_b = [&](const char * k) {
        if (cfg[k]) { this->set_parameter(rclcpp::Parameter(k, cfg[k].as<bool>())); }
      };
      set_s("aim_mode"); set_s("enemy_color");
      set_d("direct_gain"); set_d("direct_pitch_gain"); set_d("direct_ki");
      set_d("direct_yaw_bias"); set_d("direct_pitch_bias");
      set_d("cmd_smooth_alpha"); set_d("cmd_slew_deg_per_s");
      set_d("cmd_yaw_deadband_deg"); set_d("cmd_pitch_deadband_deg");
      set_b("auto_aim_enable"); set_b("allow_fire"); set_b("require_mode33");
    }

    detector_ = std::make_unique<aa::ArmorDetector>(cfg["armor_detector"]);
    tracker_ = std::make_unique<aa::ArmorTracker>(cfg["armor_tracker"]);
    fsm_ = std::make_unique<aa::AutoAimFsmController>(cfg["auto_aim_fsm"]);
    aimer_ = std::make_unique<aa::VeryAimer>(cfg["very_aimer"]);

    RCLCPP_INFO(this->get_logger(), "auto_aim inited: enemy=%s bullet=%.1f config=%s",
      enemy_color_.c_str(), bullet_speed_, config_path.c_str());

    // ---- ROS wiring ----
    const auto sensor_qos = rclcpp::SensorDataQoS();
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "image_raw", sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { on_image(msg); });
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { on_camera_info(msg); });
    vision_sub_ = this->create_subscription<auto_aim_interfaces::msg::Vision>(
      "Vision_data", rclcpp::QoS(10),
      [this](auto_aim_interfaces::msg::Vision::ConstSharedPtr msg) { on_vision(msg); });
    debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("debug_image", rclcpp::SensorDataQoS());
    ctrl_pub_ = this->create_publisher<auto_aim_interfaces::msg::RobotCtrl>(
      "Robot_ctrl_data", 10);

    // solver loop at 200 Hz (matches downlink budget at 115200 baud)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
      [this]() { solver_tick(); });
  }

private:
  // ------------------------------------------------------------------
  // input callbacks
  // ------------------------------------------------------------------
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    if (msg->k.empty() || msg->k.size() != 9) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "camera_info has no valid intrinsics");
      return;
    }
    cam_.camera_matrix = cv::Mat(3, 3, CV_64F);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        cam_.camera_matrix.at<double>(r, c) = msg->k[r * 3 + c];
      }
    }
    cam_.distortion_coefficients = cv::Mat(1, 5, CV_64F);
    for (int i = 0; i < 5 && i < static_cast<int>(msg->d.size()); ++i) {
      cam_.distortion_coefficients.at<double>(0, i) = msg->d[i];
    }
    cam_ready_ = true;
  }

  void on_vision(auto_aim_interfaces::msg::Vision::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);
    latest_vision_.id = msg->id;
    latest_vision_.mode = msg->mode;
    latest_vision_.yaw = msg->yaw;            // deg
    latest_vision_.pitch = msg->pitch;        // deg
    latest_vision_.roll = msg->roll;
    latest_vision_.quat[0] = msg->quaternion[0];
    latest_vision_.quat[1] = msg->quaternion[1];
    latest_vision_.quat[2] = msg->quaternion[2];
    latest_vision_.quat[3] = msg->quaternion[3];
    latest_vision_.shoot_speed = msg->shoot_speed;
    latest_vision_.stamp = this->now();
    vision_hist_.push_back(latest_vision_);
    if (vision_hist_.size() > 400) { vision_hist_.pop_front(); }
    ++vision_count_;
  }

  void on_image(sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    if (!cam_ready_) {
      return;
    }
    const bool aim_direct_img = this->get_parameter("aim_mode").as_string() == "direct";
    cv::Mat bgr;
    const std::string & enc = msg->encoding;
    const int h = static_cast<int>(msg->height);
    const int w = static_cast<int>(msg->width);
    const size_t step = msg->step;
    if (enc == "rgb8" && !msg->data.empty()) {
      cv::Mat rgb(h, w, CV_8UC3, const_cast<uint8_t *>(msg->data.data()), step);
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    } else if (enc == "bgr8" && !msg->data.empty()) {
      cv::Mat bgr_view(h, w, CV_8UC3, const_cast<uint8_t *>(msg->data.data()), step);
      bgr = bgr_view.clone();
    } else if (enc == "mono8" && !msg->data.empty()) {
      cv::Mat gray(h, w, CV_8UC1, const_cast<uint8_t *>(msg->data.data()), step);
      cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "unsupported image encoding: %s", enc.c_str());
      return;
    }
    if (bgr.empty()) { return; }

    const int seq = frame_seq_++;
    awakening::CommonFrame frame;
    frame.id = seq;
    frame.frame_id = seq;
    frame.img_frame.src_img = bgr;
    frame.img_frame.format = awakening::PixelFormat::BGR;
    frame.img_frame.timestamp = to_timepoint(msg->header.stamp);
    last_img_stamp_ = msg->header.stamp;
    if ((seq % 30) == 0) {
      const cv::Scalar m = cv::mean(bgr);
      RCLCPP_INFO(this->get_logger(), "[auto_aim] snap frame=%d meanBGR=(%.0f,%.0f,%.0f)",
        seq, m[0], m[1], m[2]);
      cv::imwrite("/home/nvidia/game_v2/dev/autoaim_snap.jpg", bgr);
    }

    aa::Armors armors;
    armors.timestamp = frame.img_frame.timestamp;
    armors.id = seq;
    armors.frame_id = seq;

    const cv::Rect full(0, 0, bgr.cols, bgr.rows);
    try {
      auto [lights, armor_list] = detector_->detect(frame, full, std::nullopt);
      for (auto & a : armor_list) {
        if (a.color == enemy_) { armors.armors.push_back(a); }
      }
      for (auto & l : lights) {
        if (l.color == enemy_) { armors.lights.push_back(l); }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "detect exception: %s", e.what());
      return;
    }

    // B1: static camera pose (camera frame == world Z). B2 will replace with
    // pose built from latest uplink yaw/pitch (gimbal feedback).
    const awakening::ISO3 camera_cv_in_odom = build_camera_in_z();
    (void)camera_cv_in_odom; // used below by tracker

    // ---- bench diagnostics (throttled) ----
    if (!armors.armors.empty()) { ++det_frames_; }
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[auto_aim] frame=%d armors=%zu lights=%zu det_frames=%zu",
      seq, armors.armors.size(), armors.lights.size(), det_frames_);
    if (!armors.armors.empty()) {
      const aa::Armor & a0 = armors.armors.front();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "[auto_aim]   first: color=%s number=%s",
        aa::string_by_armor_color(a0.color).c_str(),
        aa::string_by_armor_class(a0.number).c_str());
    }
    // annotated debug snapshot (every 30 frames): camera principal point + detected armors
    if ((seq % 30) == 0 && cam_ready_) {
      cv::Mat dbg = bgr.clone();
      const double cx = cam_.camera_matrix.at<double>(0, 2);
      const double cy = cam_.camera_matrix.at<double>(1, 2);
      cv::circle(dbg, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 12,
                 cv::Scalar(0, 255, 0), 2);
      cv::line(dbg, cv::Point(static_cast<int>(cx) - 25, static_cast<int>(cy)),
               cv::Point(static_cast<int>(cx) + 25, static_cast<int>(cy)), cv::Scalar(0, 255, 0), 1);
      cv::line(dbg, cv::Point(static_cast<int>(cx), static_cast<int>(cy) - 25),
               cv::Point(static_cast<int>(cx), static_cast<int>(cy) + 25), cv::Scalar(0, 255, 0), 1);
      for (auto & a : armors.armors) {
        const cv::Rect2f r = a.key_points.bounding_box();
        cv::rectangle(dbg, r, cv::Scalar(0, 255, 255), 2);
        cv::putText(dbg, aa::string_by_armor_class(a.number), cv::Point(r.x, r.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 1);
      }
      cv::imwrite("/home/nvidia/game_v2/dev/autoaim_dbg.jpg", dbg);
    }
    // ---- live debug overlay -> /auto_aim/debug_image (half res) ----
    if (cam_ready_) {
      cv::Mat dbg;
      cv::resize(bgr, dbg, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
      const double cx = cam_.camera_matrix.at<double>(0, 2) * 0.5;
      const double cy = cam_.camera_matrix.at<double>(1, 2) * 0.5;
      cv::circle(dbg, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 8,
                 cv::Scalar(0, 255, 0), 2);
      // top-left: latest downlink cmd yaw/pitch (Robot_ctrl_data)
      {
        char buf[96];
        std::snprintf(buf, sizeof(buf), "cmd yaw=%7.2f pitch=%7.2f",
                      last_ctrl_yaw_.load(), last_ctrl_pitch_.load());
        cv::putText(dbg, buf, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        cv::putText(dbg, buf, cv::Point(8, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      }
      // fps: EMA of actual on_image arrival rate
      {
        const auto now_tp = std::chrono::steady_clock::now();
        if (last_fps_tp_.time_since_epoch().count() != 0) {
          const double dt = std::chrono::duration<double>(now_tp - last_fps_tp_).count();
          if (dt > 1e-4 && dt < 1.0) {
            const double inst = 1.0 / dt;
            fps_ema_ = (fps_ema_ <= 0.0) ? inst : fps_ema_ * 0.9 + inst * 0.1;
          }
        }
        last_fps_tp_ = now_tp;
      }
      {
        char fbuf[64];
        std::snprintf(fbuf, sizeof(fbuf), "fps=%5.1f", fps_ema_);
        cv::putText(dbg, fbuf, cv::Point(8, 52), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 0, 0), 4, cv::LINE_AA);
        cv::putText(dbg, fbuf, cv::Point(8, 52), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
      }
      for (auto & a : armors.armors) {
        const cv::Rect2f r = a.key_points.bounding_box();
        cv::Rect2f rs(r.x * 0.5f, r.y * 0.5f, r.width * 0.5f, r.height * 0.5f);
        cv::rectangle(dbg, rs, cv::Scalar(0, 255, 255), 2);
        cv::putText(dbg, aa::string_by_armor_class(a.number),
                    cv::Point(static_cast<int>(rs.x), static_cast<int>(rs.y) - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
      }
      auto msg = std::make_shared<sensor_msgs::msg::Image>();
      msg->header.stamp = this->now();
      msg->header.frame_id = "camera_optical_frame";
      msg->encoding = "bgr8";
      msg->height = static_cast<uint32_t>(dbg.rows);
      msg->width = static_cast<uint32_t>(dbg.cols);
      msg->step = static_cast<uint32_t>(dbg.step);
      msg->data.assign(dbg.data, dbg.data + dbg.total() * dbg.elemSize());
      debug_pub_->publish(*msg);
    }

    // ---- direct-mode observation: keep the largest (front) armor ----
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (armors.armors.empty()) {
        direct_.valid = false;
      } else {
        // pick the armor closest to the image principal point (front plate)
        const double pcx = cam_.camera_matrix.at<double>(0, 2);
        const double pcy = cam_.camera_matrix.at<double>(1, 2);
        auto * best = &armors.armors[0];
        double best_d = 1e18;
        for (auto & a : armors.armors) {
          const cv::Rect2f r = a.key_points.bounding_box();
          const double ccx = r.x + r.width * 0.5;
          const double ccy = r.y + r.height * 0.5;
          const double dd = (ccx - pcx) * (ccx - pcx) + (ccy - pcy) * (ccy - pcy);
          if (dd < best_d) { best_d = dd; best = &a; }
        }
        const cv::Rect2f r = best->key_points.bounding_box();
        direct_.valid = true;
        direct_.x = r.x + r.width * 0.5f;
        direct_.y = r.y + r.height * 0.5f;
        direct_.w = r.width;
        direct_.h = r.height;
        direct_.t = this->now();
      }
    }
    if (!aim_direct_img) {
      aa::ArmorTarget t = tracker_->track(armors, cam_, camera_cv_in_odom, seq);
      if (t.is_inited) {
        std::lock_guard<std::mutex> lock(mtx_);
        target_ = t;
        const auto & st = target_.get_target_state();
        fsm_->update(st.vyaw(), target_.jumped, st.timestamp);
        ++track_count_;
      }
    }
  }

  /// Build pose of the camera frame (OpenCV convention) in the fixed world Z,
  /// from the latest uplink gimbal/chassis feedback (/Vision_data).
  /// Convention: world Z axes = physics (x forward, y left, z up); uplink yaw
  /// left-positive around z, pitch up-positive around y (tunable via params).
  /// Falls back to identity when feedback is disabled or never received.
  awakening::ISO3 build_camera_in_z()
  {
    std::lock_guard<std::mutex> lock(mtx_);
    const awakening::ISO3 identity = awakening::ISO3::Identity();
    if (!use_vision_feedback_ || vision_count_ == 0) { return identity; }
    // Pair the uplink gimbal state closest in time to this image frame.
    VisionSnap v = latest_vision_;
    if (!vision_hist_.empty()) {
      const double delay_s = this->get_parameter("vision_delay_ms").as_double() * 1e-3;
      const rclcpp::Time want = last_img_stamp_ - rclcpp::Duration::from_seconds(delay_s);
      auto best = vision_hist_.front();
      double best_d = 1e18;
      for (const auto & s : vision_hist_) {
        const double d = std::abs((s.stamp - want).seconds());
        if (d < best_d) { best_d = d; best = s; }
      }
      v = best;
    }
    const double d2r = M_PI / 180.0;
    const double yaw = d2r * (yaw_left_positive_ ? v.yaw : -v.yaw);
    const double pitch = d2r * (pitch_up_positive_ ? v.pitch : -v.pitch);
    const Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const Eigen::Matrix3d Rp = Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    awakening::ISO3 T = identity;
    T.linear() = Rz * (Rp * awakening::R_CV2PHYSICS);
    const Eigen::Vector3d off(cam_offset_[0], cam_offset_[1], cam_offset_[2]);
    T.translation() = Rz * off;  // chassis yaw about world z with camera offset
    return T;
  }
  // ------------------------------------------------------------------
  // solver
  // ------------------------------------------------------------------
  void solver_tick()
  {
    // read live params (allows runtime `ros2 param set` to enable/disable)
    auto_aim_enabled_ = this->get_parameter("auto_aim_enable").as_bool();
    allow_fire_ = this->get_parameter("allow_fire").as_bool();
    require_mode33_ = this->get_parameter("require_mode33").as_bool();
    aimer_->set_operator_offset(std::make_pair(
      this->get_parameter("operator_yaw_offset").as_double(),
      this->get_parameter("operator_pitch_offset").as_double()));
    int mode = 0;
    float fb_yaw = 0.0f;
    float fb_pitch = 0.0f;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      mode = latest_vision_.mode;
      fb_yaw = latest_vision_.yaw;
      fb_pitch = latest_vision_.pitch;
    }
    const bool mode33 = (mode == 33);
    const bool enabled = auto_aim_enabled_ && (!require_mode33_ || mode33);
    const aa::AutoAimFsm fsm_use = this->get_parameter("aim_center_force").as_bool()
      ? aa::AutoAimFsm::AIM_WHOLE_CAR_CENTER : fsm_->get_state();
    aa::ArmorTarget target;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      target = target_;
    }
    const bool t_ok = target.is_inited && target.check();

    auto_aim_interfaces::msg::RobotCtrl out;
    out.target_lock = 50;   // unlock default
    out.fire_command = 0;

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "[auto_aim] solver mode33=%d enabled=%d valid=%d engaged=%d",
      mode33 ? 1 : 0,
      enabled ? 1 : 0,
      t_ok ? 1 : 0, engaged_ ? 1 : 0);
    if (enabled && target.is_inited && target.check()) {
      const awakening::ISO3 shoot_in_gimbal_odom = awakening::ISO3::Identity();
      const awakening::ISO3 gimbal_in_gimbal_odom = awakening::ISO3::Identity();
      awakening::GimbalCmd cmd;
      try {
        cmd = aimer_->very_aim(
          target, bullet_speed_, fsm_use,
          shoot_in_gimbal_odom, gimbal_in_gimbal_odom);
      } catch (const std::exception & e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[auto_aim] very_aim exception: %s", e.what());
      }
      if (cmd.is_valid() && cmd.appear) {
        // exponential smoothing on the commanded yaw/pitch to reject 1-2 frame spikes
        const double alpha = this->get_parameter("cmd_smooth_alpha").as_double();
        double fy = cmd.yaw;
        double fp = cmd.pitch;
        if (std::isnan(prev_fy_)) {
          prev_fy_ = fy; prev_fp_ = fp;
        } else {
          double dy = std::remainder(fy - prev_fy_, 360.0);
          prev_fy_ = prev_fy_ + alpha * dy;
          if (prev_fy_ > 180.0) { prev_fy_ -= 360.0; }
          if (prev_fy_ < -180.0) { prev_fy_ += 360.0; }
          prev_fp_ = prev_fp_ + alpha * (fp - prev_fp_);
        }
        last_cmd_ = cmd;
        last_cmd_.yaw = prev_fy_;
        last_cmd_.pitch = prev_fp_;
        engaged_ = true;
        last_valid_tp_ = std::chrono::steady_clock::now();
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[auto_aim] very_aim invalid: valid=%d appear=%d",
          cmd.is_valid() ? 1 : 0, cmd.appear ? 1 : 0);
      }
    }

    if (!enabled) {
      engaged_ = false;
      out.yaw = 0.0f; out.pitch = 0.0f;
      out.yaw_vel = 0.0f; out.pitch_vel = 0.0f;
      out.yaw_acc = 0.0f; out.pitch_acc = 0.0f;
      out.target_lock = 50;
      out.fire_command = 0;
    } else if (engaged_ && last_cmd_.appear) {
      // locked: live command while target valid; otherwise hold position (fire/feedforward off)
      out.target_lock = 49;
      out.yaw = static_cast<float>(last_cmd_.yaw);
      out.pitch = static_cast<float>(
        last_cmd_.pitch + this->get_parameter("pitch_zero_offset_deg").as_double());
      if (t_ok) {
        out.yaw_vel = static_cast<float>(last_cmd_.v_yaw);
        out.pitch_vel = static_cast<float>(last_cmd_.v_pitch);
        out.yaw_acc = static_cast<float>(last_cmd_.a_yaw);
        out.pitch_acc = static_cast<float>(last_cmd_.a_pitch);
        out.fire_command = (allow_fire_ && last_cmd_.fire_advice) ? 1 : 0;
      } else {
        out.yaw_vel = 0.0f; out.pitch_vel = 0.0f;
        out.yaw_acc = 0.0f; out.pitch_acc = 0.0f;
        out.fire_command = 0;
      }
    } else {
      // enabled but nothing valid yet: stay unlocked, zeros
      out.target_lock = 50;
      out.yaw = 0.0f; out.pitch = 0.0f;
      out.yaw_vel = 0.0f; out.pitch_vel = 0.0f;
      out.yaw_acc = 0.0f; out.pitch_acc = 0.0f;
      out.fire_command = 0;
    }
    const bool aim_direct = this->get_parameter("aim_mode").as_string() == "direct";
    if (aim_direct) {
      const double g = this->get_parameter("direct_gain").as_double();
      const double yb = this->get_parameter("direct_yaw_bias").as_double();
      const double pb = this->get_parameter("direct_pitch_bias").as_double();
      const double R2D = 180.0 / M_PI;
      DirectObs ob;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        ob = direct_;
      }
      auto_aim_interfaces::msg::RobotCtrl d;
      d.target_lock = 50;
      d.fire_command = 0;
      if (enabled && cam_ready_ && ob.valid) {
        const double fx = cam_.camera_matrix.at<double>(0, 0);
        const double fy = cam_.camera_matrix.at<double>(1, 1);
        const double cx = cam_.camera_matrix.at<double>(0, 2);
        const double cy = cam_.camera_matrix.at<double>(1, 2);
        const double age = (this->now() - ob.t).seconds();
        if (age < 0.30) {
          const double dx_deg = -(ob.x - static_cast<float>(cx)) / fx * R2D;
          const double dy_deg = -(ob.y - static_cast<float>(cy)) / fy * R2D;
          const double pg = this->get_parameter("direct_pitch_gain").as_double();
          const double alpha = this->get_parameter("cmd_smooth_alpha").as_double();
          double fy = fb_yaw + g * dx_deg + yb;
          double fp = fb_pitch + pg * dy_deg + pb;
          if (std::isnan(prev_fy_)) {
            prev_fy_ = fy; prev_fp_ = fp;
          } else {
            double dw = std::remainder(fy - prev_fy_, 360.0);
            prev_fy_ = prev_fy_ + alpha * dw;
            if (prev_fy_ > 180.0) { prev_fy_ -= 360.0; }
            if (prev_fy_ < -180.0) { prev_fy_ += 360.0; }
            prev_fp_ = prev_fp_ + alpha * (fp - prev_fp_);
          }
          const double ki = this->get_parameter("direct_ki").as_double();
          int_y_ += dx_deg * 0.005;
          int_p_ += dy_deg * 0.005;
          int_y_ = std::max(-8.0, std::min(8.0, int_y_));
          int_p_ = std::max(-8.0, std::min(8.0, int_p_));
          fy = prev_fy_ + ki * int_y_;
          fp = prev_fp_ + ki * int_p_;
          prev_fy_ = fy; prev_fp_ = fp;
          d.target_lock = 49;
          d.yaw = static_cast<float>(fy);
          d.pitch = static_cast<float>(fp);
          const double fscale = this->get_parameter("direct_fire_scale").as_double();
          double hy = std::abs(((ob.w * 0.5) / fx) * R2D * fscale);
          double hp = std::abs(((ob.h * 0.5) / fy) * R2D * fscale);
          hy = std::max(0.2, std::min(20.0, hy));
          hp = std::max(0.2, std::min(20.0, hp));
          const bool in_win = (std::abs(dx_deg) < hy) && (std::abs(dy_deg) < hp);
          d.fire_command = (allow_fire_ && in_win) ? 2 : 0;
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[auto_aim] direct fire: errY=%.2f errP=%.2f winY=%.2f winP=%.2f fire=%d allow=%d",
            dx_deg, dy_deg, hy, hp, d.fire_command, allow_fire_ ? 1 : 0);
          last_direct_ = d;
        } else {
          // stale observation: hold last commanded pose, no fire
          d = last_direct_;
          d.yaw_vel = 0.0f; d.pitch_vel = 0.0f;
          d.yaw_acc = 0.0f; d.pitch_acc = 0.0f;
          d.fire_command = 0;
        }
      } else {
        int_y_ = 0.0; int_p_ = 0.0;
        d.target_lock = 50;
      }
      out = d;
      engaged_ = (out.target_lock == 49);
    }
    if (enabled && engaged_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[auto_aim] cmd_yaw=%.2f cmd_pitch=%.2f | fb_yaw=%.2f fb_pitch=%.2f lock=%d fsm=%s vyaw=%.2f",
        out.yaw, out.pitch, fb_yaw, fb_pitch, out.target_lock,
        aa::string_by_auto_aim_fsm(fsm_use).c_str(),
        t_ok ? target.get_target_state().vyaw() : 0.0);
    }
    if (out.target_lock == 49) {
      const double maxp = this->get_parameter("cmd_pitch_max_deg").as_double();
      const double maxy = this->get_parameter("cmd_yaw_max_deg").as_double();
      const double slew = this->get_parameter("cmd_slew_deg_per_s").as_double();
      const double step = slew * 0.005;   // 200 Hz solver
      double dy = out.yaw - fb_yaw;
      double dp = out.pitch - fb_pitch;
      dy = std::max(-step, std::min(step, dy));
      dp = std::max(-step, std::min(step, dp));
      out.yaw = static_cast<float>(fb_yaw + dy);
      out.pitch = static_cast<float>(fb_pitch + dp);
      out.pitch = std::max(-static_cast<float>(maxp), std::min(static_cast<float>(maxp), out.pitch));
      out.yaw = std::max(-static_cast<float>(maxy), std::min(static_cast<float>(maxy), out.yaw));
      // limit per-axis excursion from current feedback to suppress spikey plate swaps
      const double dp_max = this->get_parameter("cmd_pitch_delta_max_deg").as_double();
      const double dy_max = this->get_parameter("cmd_yaw_delta_max_deg").as_double();
      const float dp_delta = out.pitch - fb_pitch;
      const float dyf = out.yaw - fb_yaw;
      out.pitch = fb_pitch + std::max(-static_cast<float>(dp_max),
                                      std::min(static_cast<float>(dp_max), dp_delta));
      out.yaw = fb_yaw + std::max(-static_cast<float>(dy_max),
                                  std::min(static_cast<float>(dy_max), dyf));
    }
    // deadband: do not forward tiny yaw/pitch updates to the MCU (keep last sent)
    if (out.target_lock == 49) {
      const double ydb = this->get_parameter("cmd_yaw_deadband_deg").as_double();
      const double pdb = this->get_parameter("cmd_pitch_deadband_deg").as_double();
      if (std::isnan(last_sent_yaw_)) {
        last_sent_yaw_ = out.yaw;
        last_sent_pitch_ = out.pitch;
      } else {
        const double dy = std::remainder(static_cast<double>(out.yaw) - last_sent_yaw_, 360.0);
        const double dp = static_cast<double>(out.pitch) - last_sent_pitch_;
        if (std::abs(dy) < ydb) { out.yaw = static_cast<float>(last_sent_yaw_); }
        else { last_sent_yaw_ = static_cast<double>(out.yaw); }
        if (std::abs(dp) < pdb) { out.pitch = static_cast<float>(last_sent_pitch_); }
        else { last_sent_pitch_ = static_cast<double>(out.pitch); }
      }
    } else {
      last_sent_yaw_ = std::nan("");
      last_sent_pitch_ = std::nan("");
    }

    if (engaged_ != last_engaged_log_) {
      last_engaged_log_ = engaged_;
      RCLCPP_INFO(this->get_logger(), "[auto_aim] LOCK_STATE %s (mode33=%d valid=%d)",
        engaged_ ? "LOCK" : "UNLOCK", mode33 ? 1 : 0,
        (target.is_inited && target.check()) ? 1 : 0);
    }
    last_ctrl_yaw_.store(out.yaw);
    last_ctrl_pitch_.store(out.pitch);
    ctrl_pub_->publish(out);
  }

  // ------------------------------------------------------------------
  // members
  // ------------------------------------------------------------------
  struct VisionSnap
  {
    rclcpp::Time stamp;
    uint16_t id = 0;
    uint16_t mode = 0;
    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float shoot_speed = 0.0f;
  };

  struct DirectObs
  {
    bool valid = false;
    float x = 0.0f, y = 0.0f;   // armor center px
    float w = 0.0f, h = 0.0f;   // bbox px
    rclcpp::Time t;
  };

  std::unique_ptr<aa::ArmorDetector> detector_;
  std::unique_ptr<aa::ArmorTracker> tracker_;
  std::unique_ptr<aa::AutoAimFsmController> fsm_;
  std::unique_ptr<aa::VeryAimer> aimer_;

  awakening::CameraInfo cam_;
  bool cam_ready_ = false;
  aa::ArmorColor enemy_ = aa::ArmorColor::BLUE;
  std::string enemy_color_ = "blue";
  double bullet_speed_ = 11.0;
  std::atomic<bool> auto_aim_enabled_ {false};
  // most recently published downlink cmd (Robot_ctrl_data) for debug overlay
  std::atomic<float> last_ctrl_yaw_ {0.0f};
  std::atomic<float> last_ctrl_pitch_ {0.0f};

  int frame_seq_ = 0;
  aa::ArmorTarget target_;
  awakening::GimbalCmd last_cmd_;
  auto_aim_interfaces::msg::RobotCtrl last_direct_;
  bool engaged_ = false;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Vision>::SharedPtr vision_sub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr ctrl_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  VisionSnap latest_vision_;
  std::deque<VisionSnap> vision_hist_;
  VisionSnap pose_vision_;  // paired to latest processed image
  rclcpp::Time last_img_stamp_;
  double prev_fy_ = std::nan("");
  double prev_fp_ = std::nan("");
  double int_y_ = 0.0;
  double int_p_ = 0.0;
  double last_sent_yaw_ = std::nan("");
  double last_sent_pitch_ = std::nan("");
  std::chrono::steady_clock::time_point last_fps_tp_ {};
  double fps_ema_ = 0.0;
  size_t vision_count_ = 0;
  size_t track_count_ = 0;
  size_t det_frames_ = 0;
  DirectObs direct_;
  bool allow_fire_ = false;
  bool require_mode33_ = true;
  bool last_engaged_log_ = false;
  std::chrono::steady_clock::time_point last_valid_tp_ {};
  bool use_vision_feedback_ = true;
  bool pitch_up_positive_ = true;
  bool yaw_left_positive_ = true;
  double cam_offset_[3] = {0.0, 0.0, 0.0};
};
}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::AutoAimNode)
