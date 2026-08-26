#include "MvCameraControl.h"

#include "hik_camera/camera_safety.hpp"

// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace hik_camera
{
namespace
{

constexpr std::size_t kSerialNumberCapacity = INFO_MAX_BUFFER_SIZE;
constexpr unsigned int kCaptureTimeoutMs = 1000U;
constexpr int kMaxConsecutiveCaptureFailures = 5;

std::runtime_error sdkError(const char * operation, int status)
{
  return std::runtime_error(
    std::string(operation) + " failed with SDK status " + formatSdkStatus(status));
}

void throwIfSdkFailed(const char * operation, int status)
{
  if (status != MV_OK) {
    throw sdkError(operation, status);
  }
}

bool isParameterDouble(const rclcpp::Parameter & parameter)
{
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE;
}

bool isParameterInteger(const rclcpp::Parameter & parameter)
{
  return parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER;
}

}  // namespace

class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options)
  : Node("hik_camera", options)
  {
    RCLCPP_INFO(get_logger(), "Starting HikCameraNode");

    const bool use_sensor_data_qos =
      declare_parameter("use_sensor_data_qos", true);
    const auto qos = use_sensor_data_qos ?
      rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "/image_raw", qos);

    camera_serial_ = declare_parameter<std::string>("camera_serial", "");
    image_msg_.header.frame_id =
      declare_parameter<std::string>("frame_id", "camera_optical_frame");
    if (image_msg_.header.frame_id.empty()) {
      throw std::invalid_argument("frame_id must not be empty");
    }
    image_msg_.encoding = "rgb8";

    try {
      loadCameraInfo();
      initializeCamera();
      declareParameters();

      params_callback_handle_ = add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

      const int start_status = MV_CC_StartGrabbing(camera_handle_);
      throwIfSdkFailed("MV_CC_StartGrabbing", start_status);
      grabbing_.store(true);

      RCLCPP_INFO(
        get_logger(),
        "/image_raw.header.stamp uses this node's local ROS clock immediately before "
        "publication; no SDK hardware, IMU, or MCU timestamp is used");

      capture_thread_ = std::thread{&HikCameraNode::captureLoop, this};
    } catch (const std::exception & exception) {
      RCLCPP_FATAL(get_logger(), "Camera initialization failed: %s", exception.what());
      stopCamera();
      throw;
    } catch (...) {
      RCLCPP_FATAL(get_logger(), "Camera initialization failed with an unknown error");
      stopCamera();
      throw;
    }
  }

  ~HikCameraNode() override
  {
    stopCamera();
    RCLCPP_INFO(get_logger(), "HikCameraNode destroyed");
  }

private:
  struct ParameterRange
  {
    double minimum{0.0};
    double maximum{0.0};
  };

  struct PendingParameterWrite
  {
    std::string parameter_name;
    const char * sdk_name{nullptr};
    double value{0.0};
    float previous_value{0.0F};
  };

  void initializeCamera()
  {
    MV_CC_DEVICE_INFO_LIST device_list{};
    const int enum_status = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (enum_status != MV_OK) {
      RCLCPP_ERROR(
        get_logger(), "MV_CC_EnumDevices failed with SDK status %s",
        formatSdkStatus(enum_status).c_str());
      throw sdkError("MV_CC_EnumDevices", enum_status);
    }

    const auto max_device_count =
      sizeof(device_list.pDeviceInfo) / sizeof(device_list.pDeviceInfo[0]);
    if (device_list.nDeviceNum == 0U) {
      throw std::runtime_error("MV_CC_EnumDevices returned no USB cameras");
    }
    if (device_list.nDeviceNum > max_device_count) {
      throw std::runtime_error("MV_CC_EnumDevices returned an invalid camera count");
    }

    std::vector<std::string> serial_numbers;
    serial_numbers.reserve(device_list.nDeviceNum);
    for (unsigned int index = 0U; index < device_list.nDeviceNum; ++index) {
      const auto * device_info = device_list.pDeviceInfo[index];
      if (device_info == nullptr) {
        throw std::runtime_error("camera enumeration returned a null device entry");
      }
      if (device_info->nTLayerType != MV_USB_DEVICE) {
        throw std::runtime_error("camera enumeration returned a non-USB device");
      }

      serial_numbers.push_back(
        boundedByteString(
          device_info->SpecialInfo.stUsb3VInfo.chSerialNumber,
          kSerialNumberCapacity));
    }

    const auto selection = selectCameraDevice(serial_numbers, camera_serial_);
    if (!selection.success) {
      RCLCPP_ERROR(get_logger(), "Camera selection rejected: %s", selection.reason.c_str());
      throw std::runtime_error("camera selection rejected: " + selection.reason);
    }

    const int create_status = MV_CC_CreateHandle(
      &camera_handle_, device_list.pDeviceInfo[selection.index]);
    if (create_status != MV_OK) {
      if (camera_handle_ != nullptr) {
        const int destroy_status = MV_CC_DestroyHandle(camera_handle_);
        if (destroy_status != MV_OK) {
          RCLCPP_WARN(
            get_logger(), "Cleanup after MV_CC_CreateHandle failure returned SDK status %s",
            formatSdkStatus(destroy_status).c_str());
        }
      }
      camera_handle_ = nullptr;
      throw sdkError("MV_CC_CreateHandle", create_status);
    }
    handle_created_ = true;

    const int open_status = MV_CC_OpenDevice(camera_handle_);
    if (open_status != MV_OK) {
      throw sdkError("MV_CC_OpenDevice", open_status);
    }
    device_open_ = true;

    const int info_status = MV_CC_GetImageInfo(camera_handle_, &img_info_);
    if (info_status != MV_OK) {
      throw sdkError("MV_CC_GetImageInfo", info_status);
    }

    RCLCPP_INFO(
      get_logger(), "Selected USB camera index %zu; maximum frame %ux%u",
      selection.index, img_info_.nWidthMax, img_info_.nHeightMax);
  }

  void declareParameters()
  {
    declareExposureParameter();
    declareDoubleParameter("gain", "Gain", "Camera gain");
    declareDoubleParameter(
      "balance_ratio_r", "BalanceRatio_R", "Red balance ratio");
    declareDoubleParameter(
      "balance_ratio_g", "BalanceRatio_G", "Green balance ratio");
    declareDoubleParameter(
      "balance_ratio_b", "BalanceRatio_B", "Blue balance ratio");
  }

  MVCC_FLOATVALUE getFloatValue(const char * sdk_name)
  {
    MVCC_FLOATVALUE value{};
    const int status = MV_CC_GetFloatValue(camera_handle_, sdk_name, &value);
    if (status != MV_OK) {
      throw std::runtime_error(
              std::string("MV_CC_GetFloatValue(") + sdk_name + ") failed with SDK status " +
              formatSdkStatus(status));
    }
    if (!isFiniteInRange(value.fCurValue, value.fMin, value.fMax)) {
      throw std::runtime_error(
              std::string("SDK returned invalid range for ") + sdk_name);
    }
    return value;
  }

  void declareExposureParameter()
  {
    const auto value = getFloatValue("ExposureTime");
    if (value.fMin < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      value.fMax > static_cast<double>(std::numeric_limits<std::int64_t>::max()))
    {
      throw std::runtime_error("ExposureTime exceeds the ROS integer parameter range");
    }

    const auto minimum = static_cast<std::int64_t>(std::ceil(value.fMin));
    const auto maximum = static_cast<std::int64_t>(std::floor(value.fMax));
    if (minimum > maximum) {
      throw std::runtime_error("ExposureTime has no valid integer range");
    }

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Exposure time in microseconds";
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    range.step = 1;
    descriptor.integer_range.push_back(range);

    const auto default_exposure =
      std::max(minimum, std::min(maximum, std::int64_t{5000}));
    const auto exposure_time =
      declare_parameter<std::int64_t>("exposure_time", default_exposure, descriptor);
    if (!isFiniteInRange(
        static_cast<double>(exposure_time), value.fMin, value.fMax))
    {
      throw std::runtime_error("exposure_time is outside the SDK range");
    }

    parameter_ranges_["exposure_time"] = {value.fMin, value.fMax};
    setFloatParameter("exposure_time", "ExposureTime", static_cast<double>(exposure_time));
  }

  void declareDoubleParameter(
    const std::string & parameter_name,
    const char * sdk_name,
    const char * description)
  {
    const auto value = getFloatValue(sdk_name);

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = value.fMin;
    range.to_value = value.fMax;
    range.step = 0.0;
    descriptor.floating_point_range.push_back(range);

    const auto parameter_value =
      declare_parameter<double>(parameter_name, static_cast<double>(value.fCurValue), descriptor);
    if (!isFiniteInRange(parameter_value, value.fMin, value.fMax)) {
      throw std::runtime_error(parameter_name + " is outside the SDK range");
    }

    parameter_ranges_[parameter_name] = {value.fMin, value.fMax};
    setFloatParameter(parameter_name, sdk_name, parameter_value);
  }

  void setFloatParameter(
    const std::string & parameter_name,
    const char * sdk_name,
    double value)
  {
    const auto range = parameter_ranges_.find(parameter_name);
    if (range == parameter_ranges_.end() ||
      !isFiniteInRange(value, range->second.minimum, range->second.maximum))
    {
      throw std::runtime_error(parameter_name + " is outside the SDK range");
    }

    const int status =
      MV_CC_SetFloatValue(camera_handle_, sdk_name, static_cast<float>(value));
    if (status != MV_OK) {
      throw std::runtime_error(
              std::string("MV_CC_SetFloatValue(") + sdk_name + ") failed with SDK status " +
              formatSdkStatus(status));
    }

    RCLCPP_INFO(
      get_logger(), "%s set to %.3f", parameter_name.c_str(), value);
  }

  void loadCameraInfo()
  {
    camera_name_ = declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);

    camera_info_url_ = declare_parameter<std::string>("camera_info_url", "");
    if (camera_info_url_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "camera_info_url is empty; publishing explicitly uncalibrated CameraInfo until a "
        "verified calibration is supplied");
      return;
    }
    if (camera_info_url_ == "package://hik_camera/config/camera_info.yaml") {
      throw std::invalid_argument(
              "the checked-in camera_info.yaml is an unverified format example and cannot be "
              "loaded as formal calibration");
    }
    if (!camera_info_manager_->validateURL(camera_info_url_)) {
      throw std::invalid_argument("invalid camera_info_url: " + camera_info_url_);
    }
    if (!camera_info_manager_->loadCameraInfo(camera_info_url_)) {
      throw std::runtime_error("unable to load camera_info_url: " + camera_info_url_);
    }

    camera_info_msg_ = camera_info_manager_->getCameraInfo();
    camera_info_contract_ = CameraInfoContractSample{
      camera_info_msg_.width, camera_info_msg_.height,
      camera_info_msg_.distortion_model, camera_info_msg_.k, camera_info_msg_.d,
    };
    const auto validation = validateCameraInfoContract(camera_info_contract_);
    if (!validation.valid) {
      throw std::invalid_argument(
              "camera_info_url does not satisfy the input contract: " + validation.reason);
    }
    camera_info_calibrated_ = true;
    RCLCPP_INFO(
      get_logger(), "Loaded CameraInfo from explicit URL %s for %ux%u",
      camera_info_url_.c_str(), camera_info_msg_.width, camera_info_msg_.height);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::vector<PendingParameterWrite> pending_writes;

    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "camera_serial" ||
        parameter.get_name() == "camera_name" ||
        parameter.get_name() == "camera_info_url" ||
        parameter.get_name() == "frame_id" ||
        parameter.get_name() == "use_sensor_data_qos")
      {
        result.successful = false;
        result.reason = parameter.get_name() + " cannot be changed while the node is running";
        return result;
      }

      const auto range = parameter_ranges_.find(parameter.get_name());
      if (range == parameter_ranges_.end()) {
        continue;
      }

      double value = 0.0;
      if (parameter.get_name() == "exposure_time") {
        if (!isParameterInteger(parameter)) {
          result.successful = false;
          result.reason = "exposure_time must be an integer";
          return result;
        }
        value = static_cast<double>(parameter.as_int());
      } else {
        if (!isParameterDouble(parameter)) {
          result.successful = false;
          result.reason = parameter.get_name() + " must be a floating-point value";
          return result;
        }
        value = parameter.as_double();
      }

      if (!isFiniteInRange(value, range->second.minimum, range->second.maximum)) {
        result.successful = false;
        result.reason = parameter.get_name() + " is outside the SDK range";
        return result;
      }

      const char * sdk_name = nullptr;
      if (parameter.get_name() == "exposure_time") {
        sdk_name = "ExposureTime";
      } else if (parameter.get_name() == "gain") {
        sdk_name = "Gain";
      } else if (parameter.get_name() == "balance_ratio_r") {
        sdk_name = "BalanceRatio_R";
      } else if (parameter.get_name() == "balance_ratio_g") {
        sdk_name = "BalanceRatio_G";
      } else if (parameter.get_name() == "balance_ratio_b") {
        sdk_name = "BalanceRatio_B";
      }

      if (sdk_name == nullptr) {
        result.successful = false;
        result.reason = parameter.get_name() + " has no SDK field mapping";
        return result;
      }

      pending_writes.push_back(
        PendingParameterWrite{parameter.get_name(), sdk_name, value, 0.0F});
    }

    for (auto & write : pending_writes) {
      MVCC_FLOATVALUE current_value{};
      const int status =
        MV_CC_GetFloatValue(camera_handle_, write.sdk_name, &current_value);
      if (status != MV_OK) {
        RCLCPP_ERROR(
          get_logger(), "Unable to read %s before update; SDK status %s",
          write.parameter_name.c_str(), formatSdkStatus(status).c_str());
        result.successful = false;
        result.reason =
          write.parameter_name + " read-before-write failed with SDK status " +
          formatSdkStatus(status);
        return result;
      }
      const auto range = parameter_ranges_.at(write.parameter_name);
      if (!isFiniteInRange(
          current_value.fCurValue, range.minimum, range.maximum))
      {
        RCLCPP_ERROR(
          get_logger(), "SDK returned an invalid current value for %s",
          write.parameter_name.c_str());
        result.successful = false;
        result.reason = write.parameter_name + " returned an invalid current SDK value";
        return result;
      }
      write.previous_value = current_value.fCurValue;
    }

    for (std::size_t index = 0U; index < pending_writes.size(); ++index) {
      const auto & write = pending_writes[index];
      const int status = MV_CC_SetFloatValue(
        camera_handle_, write.sdk_name, static_cast<float>(write.value));
      if (status != MV_OK) {
        RCLCPP_ERROR(
          get_logger(), "%s write failed with SDK status %s",
          write.parameter_name.c_str(), formatSdkStatus(status).c_str());

        bool rollback_failed = false;
        for (std::size_t rollback_count = index + 1U; rollback_count > 0U; --rollback_count) {
          const auto & rollback_write = pending_writes[rollback_count - 1U];
          const int rollback_status = MV_CC_SetFloatValue(
            camera_handle_, rollback_write.sdk_name, rollback_write.previous_value);
          if (rollback_status != MV_OK) {
            rollback_failed = true;
            RCLCPP_ERROR(
              get_logger(), "Rollback of %s failed with SDK status %s",
              rollback_write.parameter_name.c_str(),
              formatSdkStatus(rollback_status).c_str());
          }
        }

        result.successful = false;
        result.reason =
          write.parameter_name + " write failed with SDK status " +
          formatSdkStatus(status);
        if (rollback_failed) {
          result.reason += "; one or more rollback writes also failed";
        }
        return result;
      }

      RCLCPP_INFO(
        get_logger(), "%s set to %.3f", write.parameter_name.c_str(), write.value);
    }

    return result;
  }

  bool publishFrame(const MV_FRAME_OUT & out_frame)
  {
    const auto width = static_cast<std::uint32_t>(out_frame.stFrameInfo.nWidth);
    const auto height = static_cast<std::uint32_t>(out_frame.stFrameInfo.nHeight);
    if (out_frame.pBufAddr == nullptr || out_frame.stFrameInfo.nFrameLen == 0U) {
      RCLCPP_ERROR(get_logger(), "Rejecting an empty SDK frame buffer");
      return false;
    }

    std::size_t buffer_size = 0U;
    if (!calculateRgb8BufferSize(width, height, buffer_size)) {
      RCLCPP_ERROR(get_logger(), "Rejecting frame with invalid dimensions %ux%u", width, height);
      return false;
    }
    if (buffer_size > std::numeric_limits<unsigned int>::max()) {
      RCLCPP_ERROR(get_logger(), "RGB8 frame is too large for the SDK conversion API");
      return false;
    }

    MV_CC_PIXEL_CONVERT_PARAM convert_param{};
    convert_param.nWidth = out_frame.stFrameInfo.nWidth;
    convert_param.nHeight = out_frame.stFrameInfo.nHeight;
    convert_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    convert_param.nDstBufferSize = static_cast<unsigned int>(buffer_size);
    convert_param.pSrcData = out_frame.pBufAddr;
    convert_param.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
    convert_param.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

    image_msg_.data.resize(buffer_size);
    convert_param.pDstBuffer = image_msg_.data.data();

    const int convert_status =
      MV_CC_ConvertPixelType(camera_handle_, &convert_param);
    if (convert_status != MV_OK) {
      RCLCPP_ERROR(
        get_logger(), "MV_CC_ConvertPixelType failed with SDK status %s",
        formatSdkStatus(convert_status).c_str());
      return false;
    }
    if (convert_param.nDstLen != buffer_size) {
      RCLCPP_ERROR(
        get_logger(), "RGB8 conversion returned %u bytes, expected %zu",
        convert_param.nDstLen, buffer_size);
      return false;
    }

    if (camera_info_calibrated_ &&
      !cameraInfoMatchesFrame(camera_info_contract_, width, height))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting %ux%u image because verified CameraInfo is for %ux%u",
        width, height, camera_info_contract_.width, camera_info_contract_.height);
      return false;
    }
    if (!camera_info_calibrated_) {
      camera_info_msg_.width = width;
      camera_info_msg_.height = height;
    }

    image_msg_.header.stamp = now();
    image_msg_.height = height;
    image_msg_.width = width;
    image_msg_.step = width * 3U;
    camera_info_msg_.header = image_msg_.header;
    camera_pub_.publish(image_msg_, camera_info_msg_);
    return true;
  }

  void captureLoop()
  {
    RCLCPP_INFO(get_logger(), "Publishing image frames");

    while (rclcpp::ok() && !stop_requested_.load()) {
      MV_FRAME_OUT out_frame{};
      const int get_status =
        MV_CC_GetImageBuffer(camera_handle_, &out_frame, kCaptureTimeoutMs);
      if (get_status != MV_OK) {
        if (stop_requested_.load() || !rclcpp::ok()) {
          break;
        }

        ++failure_count_;
        RCLCPP_WARN(
          get_logger(), "MV_CC_GetImageBuffer failed with SDK status %s (failure %d/%d)",
          formatSdkStatus(get_status).c_str(), failure_count_.load(),
          kMaxConsecutiveCaptureFailures);
        if (failure_count_ >= kMaxConsecutiveCaptureFailures) {
          RCLCPP_FATAL(get_logger(), "Camera capture failed repeatedly; stopping node");
          stop_requested_.store(true);
          rclcpp::shutdown();
        }
        continue;
      }

      bool frame_ok = false;
      try {
        frame_ok = publishFrame(out_frame);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(get_logger(), "Unhandled frame processing error: %s", exception.what());
      } catch (...) {
        RCLCPP_ERROR(get_logger(), "Unhandled frame processing error");
      }

      const int free_status = MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
      if (free_status != MV_OK) {
        RCLCPP_FATAL(
          get_logger(), "MV_CC_FreeImageBuffer failed with SDK status %s",
          formatSdkStatus(free_status).c_str());
        stop_requested_.store(true);
        rclcpp::shutdown();
        break;
      }

      if (frame_ok) {
        failure_count_ = 0;
      } else {
        ++failure_count_;
        if (failure_count_ >= kMaxConsecutiveCaptureFailures) {
          RCLCPP_FATAL(get_logger(), "Camera frame processing failed repeatedly; stopping node");
          stop_requested_.store(true);
          rclcpp::shutdown();
          break;
        }
      }
    }
  }

  void stopCamera()
  {
    stop_requested_.store(true);

    if (grabbing_.exchange(false) && camera_handle_ != nullptr) {
      const int stop_status = MV_CC_StopGrabbing(camera_handle_);
      if (stop_status != MV_OK) {
        RCLCPP_WARN(
          get_logger(), "MV_CC_StopGrabbing failed with SDK status %s",
          formatSdkStatus(stop_status).c_str());
      }
    }

    if (capture_thread_.joinable() &&
      capture_thread_.get_id() != std::this_thread::get_id())
    {
      capture_thread_.join();
    }

    if (device_open_ && camera_handle_ != nullptr) {
      const int close_status = MV_CC_CloseDevice(camera_handle_);
      if (close_status != MV_OK) {
        RCLCPP_WARN(
          get_logger(), "MV_CC_CloseDevice failed with SDK status %s",
          formatSdkStatus(close_status).c_str());
      }
      device_open_ = false;
    }

    if (handle_created_ && camera_handle_ != nullptr) {
      const int destroy_status = MV_CC_DestroyHandle(camera_handle_);
      if (destroy_status != MV_OK) {
        RCLCPP_WARN(
          get_logger(), "MV_CC_DestroyHandle failed with SDK status %s",
          formatSdkStatus(destroy_status).c_str());
      }
      camera_handle_ = nullptr;
      handle_created_ = false;
    }
  }

  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;
  image_transport::CameraPublisher camera_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  void * camera_handle_{nullptr};
  MV_IMAGE_BASIC_INFO img_info_{};
  std::string camera_serial_;
  std::string camera_name_;
  std::string camera_info_url_;
  CameraInfoContractSample camera_info_contract_;
  std::map<std::string, ParameterRange> parameter_ranges_;
  std::atomic<bool> handle_created_{false};
  std::atomic<bool> device_open_{false};
  std::atomic<bool> grabbing_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<int> failure_count_{0};
  bool camera_info_calibrated_{false};
  std::thread capture_thread_;
};

}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
