# `sp_vision_25` 旧自瞄 API 只读审计

参考目录：`/home/ubuntu22/vision-study/sp_vision_25`。本文件只记录 API 和风险，没有复制、修改或链接旧仓库代码。

本文件中的旧算法角度单位（例如 ypr 为 rad）只描述旧仓库内部接口，不能覆盖
`game_26_dev` 已确认的正式边界：串口和 ROS 位置角为 degree，只有
`auto_aim_ros2` 适配层转换为算法内部 rad。旧仓库的角度零点、坐标变换和外参均不得直接迁移。

## 实际调用链

旧离线测试 `tests/auto_aim_test.cpp` 的主链为：

```text
cv::Mat → YOLO::detect() → std::list<Armor> → Tracker::track()
        → std::list<Target> → Aimer::aim() → io::Command
```

`Solver::solve(Armor)` 在 `Tracker::set_target()` 和 `Tracker::update_target()` 内部调用。

## YOLO

文件：`tasks/auto_aim/yolo.hpp`

```cpp
class YOLOBase {
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;
  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO {
public:
  YOLO(const std::string & config_path, bool debug = true);
  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);
  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);
};
```

依赖 OpenCV、OpenVINO、yaml-cpp、旧 Detector/Classifier 和模型文件。YOLOv8/YOLO11 会整理关键点顺序；YOLOv5 的点序不能直接假设相同。

## Armor

文件：`tasks/auto_aim/armor.hpp`

```cpp
Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
Lightbar();

Armor(const Lightbar & left, const Lightbar & right);
Armor(int class_id, float confidence, const cv::Rect & box,
      std::vector<cv::Point2f> armor_keypoints);
Armor(int class_id, float confidence, const cv::Rect & box,
      std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
      std::vector<cv::Point2f> armor_keypoints);
Armor(int color_id, int num_id, float confidence, const cv::Rect & box,
      std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
```

主要字段单位：`xyz_in_gimbal/world` 为 m，`ypr_in_gimbal/world` 为 rad，`ypd_in_world` 为 yaw(rad)、pitch(rad)、distance(m)，图像点为像素。关键点数量没有完整检查，旧 class id/颜色/名称映射不是新版串口字段。

## Solver

文件：`tasks/auto_aim/solver.hpp`

```cpp
explicit Solver(const std::string & config_path);
Eigen::Matrix3d R_gimbal2world() const;
void set_R_gimbal2world(const Eigen::Quaterniond & q);
void solve(Armor & armor) const;
std::vector<cv::Point2f> reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw,
  ArmorType type, ArmorName name) const;
double oupost_reprojection_error(Armor armor, const double & pitch);
std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);
```

旧实现使用旧相机内参、相机到云台外参和 gimbal-to-world 旋转；坐标正方向不能直接带入 `game_26`。

## Tracker

文件：`tasks/auto_aim/tracker.hpp`

```cpp
Tracker(const std::string & config_path, Solver & solver);
std::string state() const;
std::list<Target> track(
  std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t,
  bool use_enemy_color = true);
std::tuple<omniperception::DetectionResult, std::list<Target>> track(
  const std::vector<omniperception::DetectionResult> & detection_queue,
  std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t,
  bool use_enemy_color = true);
```

状态机包含 `lost`、`detecting`、`tracking`、`temp_lost` 和 `switching`。Tracker 会修改输入 armors，并在目标初始化/更新时调用 Solver。

## Target

文件：`tasks/auto_aim/target.hpp`

```cpp
Target();
Target(const Armor & armor, std::chrono::steady_clock::time_point t,
       double radius, int armor_num, Eigen::VectorXd P0_dig);
Target(double x, double vyaw, double radius, double h);

void predict(std::chrono::steady_clock::time_point t);
void predict(double dt);
void update(const Armor & armor);
Eigen::VectorXd ekf_x() const;
const tools::ExtendedKalmanFilter & ekf() const;
std::vector<Eigen::Vector4d> armor_xyza_list() const;
bool diverged() const;
bool convergened();
bool checkinit();
```

旧 EKF 状态为 `[x,vx,y,vy,z,vz,angle,angular_velocity,r,l,h]`，依赖旧机器人几何和世界坐标。

## Aimer

文件：`tasks/auto_aim/aimer.hpp`

```cpp
explicit Aimer(const std::string & config_path);
io::Command aim(
  std::list<Target> targets,
  std::chrono::steady_clock::time_point timestamp,
  double bullet_speed, bool to_now = true);
io::Command aim(
  std::list<Target> targets,
  std::chrono::steady_clock::time_point timestamp,
  double bullet_speed, io::ShootMode shoot_mode,
  bool to_now = true);
```

旧 `io::Command` 只有：

```cpp
bool control;
bool shoot;
double yaw;
double pitch;
```

它没有新版速度、加速度和锁定/开火枚举。旧测试用“相邻帧 yaw 变化小于 2°”推导 `shoot=true`，不能直接套用到新版协议。

## Trajectory 与旧 Gimbal

`tools/trajectory.hpp`：

```cpp
Trajectory(double v0, double d, double h);
```

旧参数为 `v0=m/s`、`d=m`、`h=m`，内部 pitch 为“抬头为正”，Aimer 最终又取负号；新版 pitch 正方向必须重新确认。

`io/gimbal/gimbal.hpp` 使用旧的 `SP + mode + CRC16`，与新版 `0xA5 + data_length + CRC8 + cmd + payload + CRC16 + 0D0A` 完全不同，不能复用。

## 新版适配结论

当前 `game_26_dev` 新包只定义独立的：

```text
YoloStage → ArmorStage → TrackerStage → TargetStage → AimerStage
```

真实旧算法接入前仍需重新确认相机内外参、坐标方向、模型 class id、目标几何参数、弹道 pitch 符号，以及新版 `target_lock`/`fire_command` 的硬件语义。
