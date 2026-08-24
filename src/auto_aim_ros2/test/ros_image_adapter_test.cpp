#include "auto_aim_ros2/ros_image_adapter.hpp"

#include <string>

#include <gtest/gtest.h>

namespace
{
sensor_msgs::msg::Image make_image(const std::string & encoding, std::uint32_t step)
{
  sensor_msgs::msg::Image message;
  message.width = 2;
  message.height = 1;
  message.encoding = encoding;
  message.step = step;
  message.data.resize(step);
  return message;
}
}
TEST(RosImageAdapter, ConvertsRgb8ToOwnedBgr)
{
  auto message = make_image("rgb8", 6);
  message.data = {10, 20, 30, 40, 50, 60};
  std::string error;
  const auto image = rm_auto_aim::ros_adapters::to_bgr_image(message, &error);
  ASSERT_TRUE(image.has_value()) << error;
  ASSERT_EQ(image->type(), CV_8UC3);
  EXPECT_EQ(image->at<cv::Vec3b>(0, 0), cv::Vec3b(30, 20, 10));
  EXPECT_EQ(image->at<cv::Vec3b>(0, 1), cv::Vec3b(60, 50, 40));

  message.data[0] = 99;
  EXPECT_EQ(image->at<cv::Vec3b>(0, 0)[2], 10);
}

TEST(RosImageAdapter, ConvertsMono8ToThreeChannels)
{
  auto message = make_image("mono8", 2);
  message.data = {7, 9};
  const auto image = rm_auto_aim::ros_adapters::to_bgr_image(message);
  ASSERT_TRUE(image.has_value());
  EXPECT_EQ(image->at<cv::Vec3b>(0, 1), cv::Vec3b(9, 9, 9));
}

TEST(RosImageAdapter, RejectsUnsupportedEncodingAndMalformedBuffer)
{
  auto unsupported = make_image("yuv422", 4);
  std::string error;
  EXPECT_FALSE(rm_auto_aim::ros_adapters::to_bgr_image(unsupported, &error).has_value());
  EXPECT_NE(error.find("unsupported"), std::string::npos);

  auto short_step = make_image("bgr8", 5);
  error.clear();
  EXPECT_FALSE(rm_auto_aim::ros_adapters::to_bgr_image(short_step, &error).has_value());
  EXPECT_NE(error.find("step"), std::string::npos);

  auto short_data = make_image("bgr8", 6);
  short_data.data.resize(3);
  error.clear();
  EXPECT_FALSE(rm_auto_aim::ros_adapters::to_bgr_image(short_data, &error).has_value());
  EXPECT_NE(error.find("shorter"), std::string::npos);
}

TEST(RosImageAdapter, RejectsEmptyImage)
{
  sensor_msgs::msg::Image message;
  std::string error;
  EXPECT_FALSE(rm_auto_aim::ros_adapters::to_bgr_image(message, &error).has_value());
  EXPECT_NE(error.find("zero"), std::string::npos);
}
