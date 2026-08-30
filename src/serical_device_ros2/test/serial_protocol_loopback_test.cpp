#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include "crc.h"
#include "protocol_new.hpp"
#include "serial_frame_parser.h"
#include "serial_main.h"
#include "serial_ros_mapping.hpp"

namespace
{

constexpr std::size_t kFramePrefixAndCrcOverhead =
  sizeof(io::FrameHeader) + sizeof(std::uint16_t) + sizeof(std::uint16_t);
constexpr std::size_t kVisionFrameOverhead =
  kFramePrefixAndCrcOverhead + sizeof(io::MsgEndInfo);
constexpr std::size_t kRobotCtrlFrameOverhead =
  kFramePrefixAndCrcOverhead + sizeof(io::MsgEndInfo);
constexpr std::uint8_t kUntouchedByte = 0x5aU;

std::uint8_t reference_crc8(const std::uint8_t * data, std::size_t length)
{
  // The deployed table is the reflected CRC-8 polynomial 0x31, init 0xff.
  std::uint8_t crc = 0xff;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x01U) != 0U ? static_cast<std::uint8_t>((crc >> 1U) ^ 0x8cU) :
        static_cast<std::uint8_t>(crc >> 1U);
    }
  }
  return crc;
}

std::uint16_t reference_crc16(const std::uint8_t * data, std::size_t length)
{
  // The deployed table is the reflected CRC-16/CCITT polynomial 0x1021, init 0xffff.
  std::uint16_t crc = 0xffff;
  for (std::size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x0001U) != 0U ? static_cast<std::uint16_t>((crc >> 1U) ^ 0x8408U) :
        static_cast<std::uint16_t>(crc >> 1U);
    }
  }
  return crc;
}

io::VisionData make_vision()
{
  io::VisionData value{};
  value.id = 0x1234;
  value.mode = 33;
  value.yaw = 1.0F;
  value.yaw_vel = -2.5F;
  value.pitch = 0.25F;
  value.pitch_vel = 3.5F;
  value.roll = -0.75F;
  value.quaternion[0] = 0.5F;
  value.quaternion[1] = -0.5F;
  value.quaternion[2] = 0.25F;
  value.quaternion[3] = 1.0F;
  value.shoot_speed = 28.6F;
  value.bullet_count = 0x0345;
  value.game_progress = 7;
  return value;
}

io::RobotCtrlData make_control()
{
  io::RobotCtrlData value{};
  value.yaw = -1.0F;
  value.yaw_vel = 0.2F;
  value.yaw_acc = -0.3F;
  value.pitch = 0.4F;
  value.pitch_vel = -0.5F;
  value.pitch_acc = 0.6F;
  value.target_lock = 49;
  value.fire_command = 0;
  return value;
}

template<typename T>
std::uint16_t pack(
  SerialMain & serial, T payload, std::uint16_t command,
  std::array<std::uint8_t, 512> & frame)
{
  return serial.SenderPackSolve(
    reinterpret_cast<std::uint8_t *>(&payload), sizeof(T), command, frame.data());
}

void expect_tail_framed_packet(
  std::array<std::uint8_t, 512> & frame, std::uint16_t length, std::uint16_t payload_length,
  std::uint16_t command, const void * payload)
{
  ASSERT_EQ(length, payload_length + kVisionFrameOverhead);

  io::FrameHeader header{};
  std::memcpy(&header, frame.data(), sizeof(header));
  EXPECT_EQ(header.sof, io::HEADER_SOF);
  EXPECT_EQ(header.data_length, payload_length);
  EXPECT_EQ(header.seq, 1);
  EXPECT_EQ(header.crc8, reference_crc8(frame.data(), sizeof(io::FrameHeader) - 1));
  EXPECT_TRUE(Verify_CRC8_Check_Sum(frame.data(), sizeof(io::FrameHeader)));

  std::uint16_t encoded_command = 0;
  std::memcpy(
    &encoded_command, frame.data() + sizeof(io::FrameHeader), sizeof(encoded_command));
  EXPECT_EQ(encoded_command, command);

  const auto payload_offset = sizeof(io::FrameHeader) + sizeof(encoded_command);
  EXPECT_EQ(std::memcmp(frame.data() + payload_offset, payload, payload_length), 0);

  const auto crc16_offset = payload_offset + payload_length;
  const auto expected_crc16 = reference_crc16(frame.data(), crc16_offset);
  EXPECT_EQ(frame[crc16_offset], static_cast<std::uint8_t>(expected_crc16 & 0xffU));
  EXPECT_EQ(frame[crc16_offset + 1], static_cast<std::uint8_t>((expected_crc16 >> 8U) & 0xffU));
  EXPECT_TRUE(Verify_CRC16_Check_Sum(
    frame.data(), static_cast<std::uint32_t>(crc16_offset + 2)));

  EXPECT_EQ(frame[length - 2], io::END1_SOF);
  EXPECT_EQ(frame[length - 1], io::END2_SOF);
}

}  // namespace

TEST(SerialProtocolLayout, PackedOffsetsAndFrameLengths)
{
  EXPECT_EQ(sizeof(io::FrameHeader), 5U);
  EXPECT_EQ(sizeof(io::VisionData), 47U);
  EXPECT_EQ(sizeof(io::RobotCtrlData), 26U);
  EXPECT_EQ(sizeof(io::MsgEndInfo), 2U);

  EXPECT_EQ(offsetof(io::FrameHeader, crc8), 4U);
  EXPECT_EQ(offsetof(io::VisionData, quaternion), 24U);
  EXPECT_EQ(offsetof(io::VisionData, game_progress), 46U);
  EXPECT_EQ(offsetof(io::RobotCtrlData, target_lock), 24U);
  EXPECT_EQ(offsetof(io::RobotCtrlData, fire_command), 25U);

  EXPECT_EQ(sizeof(io::VisionData) + kVisionFrameOverhead, 58U);
  EXPECT_EQ(sizeof(io::RobotCtrlData) + kRobotCtrlFrameOverhead, 37U);
}

TEST(SerialCrc, KnownVectorsAndCrc16LittleEndianWireOrder)
{
  // These are fixed regression vectors for the deployed reflected lookup
  // tables (CRC-8 init 0xff and CRC-16 init 0xffff).  They are not a claim
  // that a vehicle golden frame or hardware CRC contract has been confirmed.
  std::array<std::uint8_t, 1> one_byte_zero{{0x00U}};
  std::array<std::uint8_t, 1> one_byte_one{{0x01U}};
  std::array<std::uint8_t, 4> payload{{0x01U, 0x02U, 0x03U, 0x04U}};

  EXPECT_EQ(Get_CRC8_Check_Sum(one_byte_zero.data(), one_byte_zero.size(), 0xffU), 0x35U);
  EXPECT_EQ(Get_CRC16_Check_Sum(one_byte_zero.data(), one_byte_zero.size(), 0xffffU), 0x0f87U);
  EXPECT_EQ(Get_CRC8_Check_Sum(one_byte_one.data(), one_byte_one.size(), 0xffU), 0x6bU);
  EXPECT_EQ(Get_CRC16_Check_Sum(one_byte_one.data(), one_byte_one.size(), 0xffffU), 0x1e0eU);
  EXPECT_EQ(Get_CRC8_Check_Sum(payload.data(), payload.size(), 0xffU), 0x1fU);
  EXPECT_EQ(Get_CRC16_Check_Sum(payload.data(), payload.size(), 0xffffU), 0xc66eU);

  std::array<std::uint8_t, 5> crc8_frame{{0x01U, 0x02U, 0x03U, 0x04U, 0x00U}};
  Append_CRC8_Check_Sum(crc8_frame.data(), crc8_frame.size());
  // CRC8 covers the first four bytes and occupies the final byte.
  EXPECT_EQ(crc8_frame[4], 0x1fU);
  EXPECT_TRUE(Verify_CRC8_Check_Sum(crc8_frame.data(), crc8_frame.size()));
  crc8_frame[4] ^= 0x01U;
  EXPECT_FALSE(Verify_CRC8_Check_Sum(crc8_frame.data(), crc8_frame.size()));

  std::array<std::uint8_t, 6> framed_payload{{0x01U, 0x02U, 0x03U, 0x04U, 0x00U, 0x00U}};
  Append_CRC16_Check_Sum(framed_payload.data(), framed_payload.size());
  EXPECT_EQ(framed_payload[4], 0x6eU);
  EXPECT_EQ(framed_payload[5], 0xc6U);
  EXPECT_TRUE(Verify_CRC16_Check_Sum(framed_payload.data(), framed_payload.size()));
}

TEST(SerialRosFieldMapping, VisionFieldsAreForwardedWithoutUnitConversion)
{
  const auto input = make_vision();
  const auto message = rm_auto_aim::serial_ros::to_ros_vision(input);
  EXPECT_EQ(message.id, input.id);
  EXPECT_EQ(message.mode, input.mode);
  EXPECT_FLOAT_EQ(message.yaw, input.yaw);
  EXPECT_FLOAT_EQ(message.yaw_vel, input.yaw_vel);
  EXPECT_FLOAT_EQ(message.pitch, input.pitch);
  EXPECT_FLOAT_EQ(message.pitch_vel, input.pitch_vel);
  EXPECT_FLOAT_EQ(message.roll, input.roll);
  for (std::size_t index = 0; index < 4; ++index) {
    EXPECT_FLOAT_EQ(message.quaternion[index], input.quaternion[index]);
  }
  EXPECT_FLOAT_EQ(message.shoot_speed, input.shoot_speed);
  EXPECT_EQ(message.bullet_count, input.bullet_count);
  EXPECT_EQ(message.game_progress, input.game_progress);
}

TEST(SerialRosFieldMapping, RobotCtrlFieldsAreForwardedWithoutUnitConversion)
{
  const auto input = make_control();
  auto message = auto_aim_interfaces::msg::RobotCtrl{};
  message.yaw = input.yaw;
  message.yaw_vel = input.yaw_vel;
  message.yaw_acc = input.yaw_acc;
  message.pitch = input.pitch;
  message.pitch_vel = input.pitch_vel;
  message.pitch_acc = input.pitch_acc;
  message.target_lock = input.target_lock;
  message.fire_command = input.fire_command;

  const auto output = rm_auto_aim::serial_ros::to_serial_robot_ctrl(message);
  EXPECT_FLOAT_EQ(output.yaw, input.yaw);
  EXPECT_FLOAT_EQ(output.yaw_vel, input.yaw_vel);
  EXPECT_FLOAT_EQ(output.yaw_acc, input.yaw_acc);
  EXPECT_FLOAT_EQ(output.pitch, input.pitch);
  EXPECT_FLOAT_EQ(output.pitch_vel, input.pitch_vel);
  EXPECT_FLOAT_EQ(output.pitch_acc, input.pitch_acc);
  EXPECT_EQ(output.target_lock, input.target_lock);
  EXPECT_EQ(output.fire_command, input.fire_command);
}

TEST(SerialProtocolLoopback, VisionPackAndReceiveRoundTrip)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> frame{};

  const auto length = pack(sender, input, io::VISION_ID, frame);
  expect_tail_framed_packet(frame, length, sizeof(input), io::VISION_ID, &input);

  SerialMain receiver;
  EXPECT_EQ(
    receiver.ReceiveDataSolve(frame.data()), sizeof(io::VisionData) + kVisionFrameOverhead);
  EXPECT_EQ(std::memcmp(&receiver.vision_msg_, &input, sizeof(input)), 0);
}

TEST(SerialProtocolLoopback, RobotControlPack)
{
  const auto input = make_control();
  SerialMain sender;
  std::array<std::uint8_t, 512> frame{};
  frame.fill(kUntouchedByte);

  const auto length = pack(sender, input, io::CHASSIS_CTRL_CMD_ID, frame);
  ASSERT_EQ(length, sizeof(input) + kRobotCtrlFrameOverhead);
  EXPECT_EQ(length, 37U);

  io::FrameHeader header{};
  std::memcpy(&header, frame.data(), sizeof(header));
  EXPECT_EQ(header.sof, io::HEADER_SOF);
  EXPECT_EQ(header.data_length, sizeof(input));
  EXPECT_EQ(header.seq, 1);
  EXPECT_EQ(header.crc8, reference_crc8(frame.data(), sizeof(io::FrameHeader) - 1));
  EXPECT_TRUE(Verify_CRC8_Check_Sum(frame.data(), sizeof(io::FrameHeader)));

  std::uint16_t encoded_command = 0;
  std::memcpy(
    &encoded_command, frame.data() + sizeof(io::FrameHeader), sizeof(encoded_command));
  EXPECT_EQ(encoded_command, io::CHASSIS_CTRL_CMD_ID);

  const auto payload_offset = sizeof(io::FrameHeader) + sizeof(encoded_command);
  EXPECT_EQ(std::memcmp(frame.data() + payload_offset, &input, sizeof(input)), 0);

  const auto crc16_offset = payload_offset + sizeof(input);
  ASSERT_EQ(crc16_offset, 33U);
  EXPECT_EQ(crc16_offset + sizeof(std::uint16_t) + sizeof(io::MsgEndInfo), length);
  const auto expected_crc16 = reference_crc16(frame.data(), crc16_offset);
  EXPECT_EQ(frame[crc16_offset], static_cast<std::uint8_t>(expected_crc16 & 0xffU));
  EXPECT_EQ(frame[crc16_offset + 1], static_cast<std::uint8_t>((expected_crc16 >> 8U) & 0xffU));
  EXPECT_TRUE(Verify_CRC16_Check_Sum(frame.data(), length));

  // 0x0102 carries the same 0D 0A terminator as every other frame; the MCU
  // appends it to all of its own packets (wire capture 2026-08-31).
  EXPECT_EQ(frame[length - 2], io::END1_SOF);
  EXPECT_EQ(frame[length - 1], io::END2_SOF);
}

TEST(SerialProtocolLoopback, RejectsCorruptedFramesAndInvalidInputs)
{
  auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> valid_frame{};
  ASSERT_EQ(pack(sender, input, io::VISION_ID, valid_frame), 58U);

  SerialMain receiver;
  receiver.vision_msg_ = input;

  auto bad_sof = valid_frame;
  bad_sof[0] = 0;
  EXPECT_EQ(receiver.ReceiveDataSolve(bad_sof.data()), 0U);

  auto bad_crc8 = valid_frame;
  bad_crc8[4] ^= 0x01U;
  EXPECT_EQ(receiver.ReceiveDataSolve(bad_crc8.data()), 0U);

  auto bad_crc16 = valid_frame;
  bad_crc16[54] ^= 0x01U;
  EXPECT_EQ(receiver.ReceiveDataSolve(bad_crc16.data()), 0U);

  // A 46-byte Vision payload (struct without game_progress) matches the MCU
  // wire format and must be accepted with the missing byte left at zero.
  std::array<std::uint8_t, sizeof(io::VisionData) - 1U> short_payload{};
  std::memcpy(short_payload.data(), &input, short_payload.size());
  std::array<std::uint8_t, 512> short_frame{};
  ASSERT_EQ(
    sender.SenderPackSolve(
      short_payload.data(), short_payload.size(), io::VISION_ID, short_frame.data()),
    57U);
  EXPECT_EQ(receiver.ReceiveDataSolve(short_frame.data()), 57U);
  EXPECT_EQ(std::memcmp(&receiver.vision_msg_, short_payload.data(), short_payload.size()), 0);
  EXPECT_EQ(receiver.vision_msg_.game_progress, 0U);

  std::array<std::uint8_t, 512> oversized_frame{};
  oversized_frame[0] = io::HEADER_SOF;
  oversized_frame[1] = 0xf6U;  // 502 > BUFF_LENGTH - 11
  oversized_frame[2] = 0x01U;
  EXPECT_EQ(receiver.ReceiveDataSolve(oversized_frame.data()), 0U);

  EXPECT_EQ(receiver.ReceiveDataSolve(nullptr), 0U);
  EXPECT_EQ(sender.SenderPackSolve(nullptr, sizeof(input), io::VISION_ID, valid_frame.data()), 0U);
  EXPECT_EQ(sender.SenderPackSolve(
    reinterpret_cast<std::uint8_t *>(&input), sizeof(input), io::VISION_ID, nullptr), 0U);
  EXPECT_EQ(sender.SenderPackSolve(
    valid_frame.data(), 502U, io::VISION_ID, valid_frame.data()), 0U);
}

TEST(SerialFrameParser, DistinguishesRobotControlFrameFromVisionTailFrame)
{
  const auto vision = make_vision();
  const auto control = make_control();
  SerialMain sender;
  std::array<std::uint8_t, 512> robot_frame{};
  std::array<std::uint8_t, 512> vision_frame{};
  ASSERT_EQ(pack(sender, control, io::CHASSIS_CTRL_CMD_ID, robot_frame), 37U);
  ASSERT_EQ(pack(sender, vision, io::VISION_ID, vision_frame), 58U);

  std::vector<std::uint8_t> stream;
  stream.insert(stream.end(), robot_frame.begin(), robot_frame.begin() + 37);
  stream.insert(stream.end(), vision_frame.begin(), vision_frame.begin() + 58);

  SerialFrameParser parser;
  std::vector<std::size_t> accepted_lengths;
  EXPECT_EQ(parser.Feed(
    stream.data(), stream.size(),
    [&accepted_lengths](const std::uint8_t *, std::size_t length) {
      accepted_lengths.push_back(length);
      return true;
    }), 2U);
  ASSERT_EQ(accepted_lengths.size(), 2U);
  EXPECT_EQ(accepted_lengths[0], 37U);
  EXPECT_EQ(accepted_lengths[1], 58U);
  EXPECT_EQ(parser.buffered_length(), 0U);
}

TEST(SerialFrameParser, RejectsHeaderCrc8AndWrongPayloadLengthThenRecovers)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> bad_crc8{};
  std::array<std::uint8_t, 512> bad_length{};
  std::array<std::uint8_t, 512> valid{};
  ASSERT_EQ(pack(sender, input, io::VISION_ID, bad_crc8), 58U);
  ASSERT_EQ(pack(sender, input, io::VISION_ID, bad_length), 58U);
  ASSERT_EQ(pack(sender, input, io::VISION_ID, valid), 58U);

  bad_crc8[4] ^= 0x01U;
  io::FrameHeader bad_length_header{};
  std::memcpy(&bad_length_header, bad_length.data(), sizeof(bad_length_header));
  // 46 is the valid MCU wire length; 45 is genuinely unknown.
  bad_length_header.data_length = static_cast<std::uint16_t>(sizeof(io::VisionData) - 2U);
  std::memcpy(bad_length.data(), &bad_length_header, sizeof(bad_length_header));
  Append_CRC8_Check_Sum(bad_length.data(), sizeof(io::FrameHeader));
  constexpr std::size_t kBadLengthFrameSize = sizeof(io::VisionData) - 2U + 11;

  std::vector<std::uint8_t> stream;
  stream.insert(stream.end(), bad_crc8.begin(), bad_crc8.begin() + 58);
  stream.insert(stream.end(), bad_length.begin(), bad_length.begin() + kBadLengthFrameSize);
  stream.insert(stream.end(), valid.begin(), valid.begin() + 58);

  SerialFrameParser parser;
  std::vector<std::vector<std::uint8_t>> accepted;
  EXPECT_EQ(parser.Feed(
    stream.data(), stream.size(),
    [&accepted](const std::uint8_t * data, std::size_t length) {
      accepted.emplace_back(data, data + length);
      return true;
    }), 1U);
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_EQ(accepted.front().size(), 58U);
  EXPECT_EQ(std::memcmp(accepted.front().data(), valid.data(), 58U), 0);
  EXPECT_EQ(parser.buffered_length(), 0U);
}

TEST(SerialFrameParser, RetainsTruncatedVisionFrameUntilFinalByte)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> frame{};
  ASSERT_EQ(pack(sender, input, io::VISION_ID, frame), 58U);

  SerialFrameParser parser;
  std::size_t accepted = 0;
  const auto callback = [&accepted](const std::uint8_t *, std::size_t length) {
      EXPECT_EQ(length, 58U);
      ++accepted;
      return true;
    };
  EXPECT_EQ(parser.Feed(frame.data(), 57U, callback), 0U);
  EXPECT_EQ(accepted, 0U);
  EXPECT_EQ(parser.buffered_length(), 57U);
  EXPECT_EQ(parser.Feed(frame.data() + 57U, 1U, callback), 1U);
  EXPECT_EQ(accepted, 1U);
  EXPECT_EQ(parser.buffered_length(), 0U);
}

TEST(SerialFrameParser, WaitsForHalfFrameAndAcceptsPayloadTailBytes)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> frame{};
  ASSERT_EQ(pack(sender, input, io::VISION_ID, frame), 58U);

  // Put 0D 0A inside the raw payload and recompute only the CRC16.
  frame[7] = io::END1_SOF;
  frame[8] = io::END2_SOF;
  Append_CRC16_Check_Sum(frame.data(), 56U);

  SerialFrameParser parser;
  std::vector<std::vector<std::uint8_t>> accepted;
  const auto callback = [&accepted](const std::uint8_t * data, std::size_t length) {
      accepted.emplace_back(data, data + length);
      return true;
    };

  ASSERT_EQ(parser.Feed(frame.data(), 29U, callback), 0U);
  EXPECT_EQ(accepted.size(), 0U);
  EXPECT_EQ(parser.buffered_length(), 29U);
  EXPECT_EQ(parser.Feed(frame.data() + 29U, 29U, callback), 1U);
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_EQ(accepted.front().size(), 58U);
  EXPECT_EQ(std::memcmp(accepted.front().data(), frame.data(), 58U), 0);
  EXPECT_EQ(parser.buffered_length(), 0U);
}

TEST(SerialFrameParser, AcceptsCoalescedFrames)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> first{};
  std::array<std::uint8_t, 512> second{};
  ASSERT_EQ(pack(sender, input, io::VISION_ID, first), 58U);
  ASSERT_EQ(pack(sender, input, io::VISION_ID, second), 58U);

  std::vector<std::uint8_t> coalesced;
  coalesced.insert(coalesced.end(), first.begin(), first.begin() + 58);
  coalesced.insert(coalesced.end(), second.begin(), second.begin() + 58);

  SerialFrameParser parser;
  std::size_t callback_count = 0;
  EXPECT_EQ(parser.Feed(
    coalesced.data(), coalesced.size(),
    [&callback_count](const std::uint8_t *, std::size_t length) {
      EXPECT_EQ(length, 58U);
      ++callback_count;
      return true;
    }), 2U);
  EXPECT_EQ(callback_count, 2U);
  EXPECT_EQ(parser.buffered_length(), 0U);
}

TEST(SerialFrameParser, RejectsBadFrameAndRecoversNextValidFrame)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> bad_tail{};
  std::array<std::uint8_t, 512> bad_crc{};
  std::array<std::uint8_t, 512> valid{};
  ASSERT_EQ(pack(sender, input, io::VISION_ID, bad_tail), 58U);
  ASSERT_EQ(pack(sender, input, io::VISION_ID, bad_crc), 58U);
  ASSERT_EQ(pack(sender, input, io::VISION_ID, valid), 58U);
  bad_tail[56] = 0;
  bad_crc[54] ^= 0x01U;

  std::vector<std::uint8_t> stream;
  stream.insert(stream.end(), bad_tail.begin(), bad_tail.begin() + 58);
  stream.insert(stream.end(), bad_crc.begin(), bad_crc.begin() + 58);
  stream.insert(stream.end(), valid.begin(), valid.begin() + 58);

  SerialFrameParser parser;
  std::vector<std::vector<std::uint8_t>> accepted;
  EXPECT_EQ(parser.Feed(
    stream.data(), stream.size(),
    [&accepted](const std::uint8_t * data, std::size_t length) {
      accepted.emplace_back(data, data + length);
      return true;
    }), 1U);
  ASSERT_EQ(accepted.size(), 1U);
  EXPECT_EQ(std::memcmp(accepted.front().data(), valid.data(), 58U), 0);
}

TEST(SerialFrameParser, RejectsUnknownCommand)
{
  const auto input = make_vision();
  SerialMain sender;
  std::array<std::uint8_t, 512> unknown{};
  std::array<std::uint8_t, 512> valid{};
  ASSERT_EQ(pack(sender, input, 0x9999U, unknown), 58U);
  ASSERT_EQ(pack(sender, input, io::VISION_ID, valid), 58U);

  std::vector<std::uint8_t> stream;
  stream.insert(stream.end(), unknown.begin(), unknown.begin() + 58);
  stream.insert(stream.end(), valid.begin(), valid.begin() + 58);

  SerialFrameParser parser;
  std::size_t accepted = 0;
  EXPECT_EQ(parser.Feed(
    stream.data(), stream.size(),
    [&accepted](const std::uint8_t *, std::size_t) {
      ++accepted;
      return true;
    }), 1U);
  EXPECT_EQ(accepted, 1U);
}
