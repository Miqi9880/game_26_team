#include "auto_aim_tools/ros_input_preflight_node.hpp"

#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

#include <rclcpp/node_interfaces/node_base.hpp>
#include <rclcpp/node_interfaces/node_graph.hpp>
#include <rclcpp/node_interfaces/node_timers.hpp>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/subscription_factory.hpp>

namespace auto_aim_tools
{
namespace
{
template<typename Message, typename = void>
struct HasYawAcceleration : std::false_type {};

template<typename Message>
struct HasYawAcceleration<
  Message, std::void_t<decltype(std::declval<Message>().yaw_acc)>>: std::true_type {};

template<typename Message, typename = void>
struct HasPitchAcceleration : std::false_type {};

template<typename Message>
struct HasPitchAcceleration<
  Message, std::void_t<decltype(std::declval<Message>().pitch_acc)>>: std::true_type {};

template<typename Message>
std::optional<float> yaw_acceleration(const Message & message)
{
  if constexpr (HasYawAcceleration<Message>::value) {
    return static_cast<float>(message.yaw_acc);
  }
  return std::nullopt;
}

template<typename Message>
std::optional<float> pitch_acceleration(const Message & message)
{
  if constexpr (HasPitchAcceleration<Message>::value) {
    return static_cast<float>(message.pitch_acc);
  }
  return std::nullopt;
}

HeaderStamp to_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return HeaderStamp{stamp.sec, stamp.nanosec, true};
}

template<typename Message, typename Callback>
typename rclcpp::Subscription<Message>::SharedPtr create_read_only_subscription(
  const rclcpp::node_interfaces::NodeTopics::SharedPtr & topics,
  const std::string & topic_name, Callback && callback)
{
  rclcpp::SubscriptionOptions options;
  using Subscription = rclcpp::Subscription<Message>;
  auto factory = rclcpp::create_subscription_factory<Message>(
    std::forward<Callback>(callback), options,
    Subscription::MessageMemoryStrategyType::create_default());
  auto subscription = topics->create_subscription(
    topic_name, factory, rclcpp::SensorDataQoS());
  topics->add_subscription(subscription, options.callback_group);
  return std::dynamic_pointer_cast<Subscription>(subscription);
}

}  // namespace

class RosInputPreflightNode::Impl
{
public:
  Impl(
    RosInputPreflightNode * owner, std::shared_ptr<PreflightAnalyzer> analyzer,
    rclcpp::NodeOptions options)
  : analyzer_(std::move(analyzer))
  {
    if (!analyzer_) {
      throw std::invalid_argument("analyzer must not be null");
    }

    options.enable_rosout(false);
    options.enable_topic_statistics(false);
    node_base_ = std::make_shared<rclcpp::node_interfaces::NodeBase>(
      "ros_input_preflight", "", options.context(), *options.get_rcl_node_options(),
      options.use_intra_process_comms(), false);
    node_graph_ = std::make_shared<rclcpp::node_interfaces::NodeGraph>(node_base_.get());
    node_timers_ = std::make_shared<rclcpp::node_interfaces::NodeTimers>(node_base_.get());
    node_topics_ = std::make_shared<rclcpp::node_interfaces::NodeTopics>(
      node_base_.get(), node_timers_.get());

    image_subscription_ = create_read_only_subscription<sensor_msgs::msg::Image>(
      node_topics_, kImageTopic,
      std::bind(&RosInputPreflightNode::on_image, owner, std::placeholders::_1));
    camera_info_subscription_ = create_read_only_subscription<sensor_msgs::msg::CameraInfo>(
      node_topics_, kCameraInfoTopic,
      std::bind(&RosInputPreflightNode::on_camera_info, owner, std::placeholders::_1));
    vision_subscription_ = create_read_only_subscription<auto_aim_interfaces::msg::Vision>(
      node_topics_, kVisionTopic,
      std::bind(&RosInputPreflightNode::on_vision, owner, std::placeholders::_1));
  }

  std::shared_ptr<PreflightAnalyzer> analyzer_;
  rclcpp::node_interfaces::NodeBase::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeGraph::SharedPtr node_graph_;
  rclcpp::node_interfaces::NodeTimers::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeTopics::SharedPtr node_topics_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Vision>::SharedPtr vision_subscription_;
};

RosInputPreflightNode::RosInputPreflightNode(
  std::shared_ptr<PreflightAnalyzer> analyzer, const rclcpp::NodeOptions & options)
: impl_(std::make_unique<Impl>(this, std::move(analyzer), options))
{
}

RosInputPreflightNode::~RosInputPreflightNode() = default;

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
RosInputPreflightNode::get_node_base_interface() const
{
  return impl_->node_base_;
}

rclcpp::node_interfaces::NodeGraphInterface::SharedPtr
RosInputPreflightNode::get_node_graph_interface() const
{
  return impl_->node_graph_;
}

void RosInputPreflightNode::on_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  try {
    impl_->analyzer_->observe_image(
      ImageSample{
        to_stamp(message->header.stamp), message->width, message->height,
        message->encoding, message->step, message->data.size(),
      },
      monotonic_seconds());
  } catch (const std::exception & error) {
    impl_->analyzer_->record_callback_error(kImageTopic, error.what());
  } catch (...) {
    impl_->analyzer_->record_callback_error(kImageTopic, "unknown exception");
  }
}

void RosInputPreflightNode::on_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
{
  try {
    impl_->analyzer_->observe_camera_info(
      CameraInfoSample{
        message->width, message->height, message->distortion_model,
        std::vector<double>(message->k.begin(), message->k.end()), message->d,
      },
      monotonic_seconds());
  } catch (const std::exception & error) {
    impl_->analyzer_->record_callback_error(kCameraInfoTopic, error.what());
  } catch (...) {
    impl_->analyzer_->record_callback_error(kCameraInfoTopic, "unknown exception");
  }
}

void RosInputPreflightNode::on_vision(
  const auto_aim_interfaces::msg::Vision::ConstSharedPtr & message)
{
  try {
    impl_->analyzer_->observe_vision(
      VisionSample{
        to_stamp(message->header.stamp),
        message->yaw, message->yaw_vel, yaw_acceleration(*message),
        message->pitch, message->pitch_vel, pitch_acceleration(*message),
        message->roll,
        std::vector<float>(message->quaternion.begin(), message->quaternion.end()),
        message->shoot_speed,
      },
      monotonic_seconds());
  } catch (const std::exception & error) {
    impl_->analyzer_->record_callback_error(kVisionTopic, error.what());
  } catch (...) {
    impl_->analyzer_->record_callback_error(kVisionTopic, "unknown exception");
  }
}

}  // namespace auto_aim_tools
