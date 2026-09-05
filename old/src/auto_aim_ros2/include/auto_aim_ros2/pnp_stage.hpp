#ifndef AUTO_AIM_ROS2__PNP_STAGE_HPP_
#define AUTO_AIM_ROS2__PNP_STAGE_HPP_

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "auto_aim_ros2/raw_armor_detector.hpp"

namespace rm_auto_aim::pnp
{

// solvePnP follows OpenCV's camera-optical convention: x right, y down, z
// forward.  This frame is intentionally kept separate from the confirmed
// gimbal convention (x forward, y left, z up).
inline constexpr char kCameraOpticalFrame[] = "opencv_camera_optical";
inline constexpr char kGimbalFrame[] = "gimbal_x_forward_y_left_z_up";

enum class ArmorSize : std::uint8_t { Small = 0, Large = 1 };

const char * armor_size_name(ArmorSize size) noexcept;

struct CameraCalibration
{
  int image_width{0};
  int image_height{0};
  cv::Matx33d camera_matrix{};  // pixel intrinsics; no length unit
  std::vector<double> distortion_coefficients;
  std::string source;
  std::string version;
  std::string coordinate_frame{kCameraOpticalFrame};
  bool test_only{false};

  std::optional<std::string> validate() const;
};

struct ArmorGeometry
{
  ArmorSize size{ArmorSize::Small};
  double width_m{0.0};
  double height_m{0.0};
  // Required order: top-left, top-right, bottom-right, bottom-left in the
  // explicitly named armor-local frame from the YAML configuration.
  std::array<cv::Point3d, 4> object_points_m{};
  std::string source;
  std::string version;
  std::string object_frame;
  bool test_only{false};

  std::optional<std::string> validate() const;
};

struct CameraToGimbalExtrinsic
{
  // configured=false is a safe, supported configuration: only camera-frame
  // pose is emitted.  A missing matrix is never treated as identity.
  bool configured{false};
  cv::Matx33d rotation_gimbal_from_camera{};
  cv::Vec3d translation_gimbal_from_camera_m{};
  std::string source;
  std::string version;
  std::string source_frame{kCameraOpticalFrame};
  std::string target_frame{kGimbalFrame};
  bool test_only{false};

  std::optional<std::string> validate() const;
};

struct PnpConfiguration
{
  int schema_version{0};
  bool test_only{false};
  CameraCalibration camera;
  ArmorGeometry small_armor;
  ArmorGeometry large_armor;
  // Model-specific mapping.  An unknown class with ArmorTypeHint::Unknown is
  // rejected rather than silently using a physical default size.
  std::map<int, ArmorSize> class_to_armor_size;
  double max_reprojection_error_px{0.0};
  // The WSL OpenCV 4.1 runtime supports ITERATIVE.  This value is selected
  // from an explicit YAML method name rather than silently falling back.
  int solvepnp_flag{0};
  CameraToGimbalExtrinsic camera_to_gimbal;

  const ArmorGeometry * geometry_for(
    const detector::RawArmorDetection & detection) const noexcept;
  std::optional<std::string> validate() const;
};

struct ConfigLoadOptions
{
  // The default rejects test-only calibration and geometry.  Offline tools
  // must opt in explicitly; runtime/control code must not.
  bool allow_test_only{false};
};

// Parses the versioned YAML schema and validates every required physical or
// camera field before returning.  It never reads old repository calibration.
PnpConfiguration load_pnp_configuration(
  const std::string & yaml_path,
  ConfigLoadOptions options = {});

enum class PoseFailure : std::uint8_t
{
  None = 0,
  InvalidRawDetection,
  GeometryNotConfigured,
  GeometrySemanticConflict,
  InvalidConfiguration,
  ImageDimensionsMismatch,
  KeypointOrderRejected,
  SolvePnpFailed,
  NonPositiveCameraDepth,
  ReprojectionErrorTooLarge,
  RelativeAngleUndefined,
};

const char * pose_failure_name(PoseFailure failure) noexcept;

struct RelativeAngles
{
  // Logs only.  They are not RobotCtrl angles: their absolute-vs-relative
  // command semantics remain unconfirmed.
  double relative_yaw_rad{0.0};    // +left in the confirmed gimbal frame
  double relative_pitch_rad{0.0};  // +up in the confirmed gimbal frame
};

struct PoseObservation
{
  detector::RawArmorDetection raw_detection{};
  ArmorSize armor_size{ArmorSize::Small};
  bool valid{false};
  PoseFailure failure{PoseFailure::InvalidConfiguration};

  // Valid only when valid=true; unit is m; frame is opencv_camera_optical.
  cv::Vec3d translation_in_camera_m{};
  // Rotation which maps armor-local vectors into opencv_camera_optical.
  cv::Matx33d rotation_camera_from_armor{};
  double reprojection_error_px{0.0};  // RMS over four image points

  // Populated only when a validated configured extrinsic was supplied.
  std::optional<cv::Vec3d> translation_in_gimbal_m;
  std::optional<cv::Matx33d> rotation_gimbal_from_armor;
  std::optional<RelativeAngles> relative_angles_in_gimbal;
};

class PnpStage final
{
public:
  explicit PnpStage(PnpConfiguration config);

  // Does not publish RobotCtrl or create yaw/pitch commands.  Every error is
  // represented as an invalid observation, so offline input cannot crash the
  // data path.
  PoseObservation solve(const detector::RawArmorDetection & detection) const noexcept;

  // Use this overload whenever the producing frame dimensions are available.
  // It fails closed instead of scaling K or assuming that an image was
  // resized.  The no-size overload is retained for synthetic unit tests and
  // uses the dimensions declared in the validated configuration.
  PoseObservation solve(
    const detector::RawArmorDetection & detection,
    int image_width,
    int image_height) const noexcept;

  const PnpConfiguration & config() const noexcept;

  static std::optional<RelativeAngles> relative_angles_from_gimbal_translation(
    const cv::Vec3d & translation_gimbal_m) noexcept;

  static cv::Mat annotate(
    const cv::Mat & bgr_image,
    const std::vector<PoseObservation> & observations);

private:
  PnpConfiguration config_;
};

}  // namespace rm_auto_aim::pnp

#endif  // AUTO_AIM_ROS2__PNP_STAGE_HPP_
