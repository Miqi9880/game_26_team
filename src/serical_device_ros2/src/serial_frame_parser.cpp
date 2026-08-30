#include "serial_frame_parser.h"

#include <cstring>

#include "crc.h"
#include "protocol_new.hpp"

namespace
{
constexpr std::size_t kFrameCrcOverhead =
  SerialFrameParser::kFrameOverhead - sizeof(io::MsgEndInfo);
}

bool SerialFrameParser::IsKnownPayloadLength(
  std::uint16_t command, std::uint16_t payload_length)
{
  switch (command) {
    case io::VISION_ID:
      // The MCU sends 46 bytes (vision data without the trailing
      // game_progress byte); the full 47-byte layout is also accepted.
      return payload_length == sizeof(io::VisionData) ||
        payload_length == sizeof(io::VisionData) - 1;
    case io::CHASSIS_CTRL_CMD_ID:
      return payload_length == sizeof(io::RobotCtrlData);
    default:
      return false;
  }
}

bool SerialFrameParser::HasFrameTail(std::uint16_t command) noexcept
{
  // Both directions carry the 0D 0A terminator: the MCU appends it to every
  // frame (wire capture 2026-08-31) and the sender mirrors it.
  return command == io::VISION_ID || command == io::CHASSIS_CTRL_CMD_ID;
}

void SerialFrameParser::DiscardPrefix(std::size_t length) noexcept
{
  if (length >= buffered_length_) {
    buffered_length_ = 0;
    return;
  }

  std::memmove(
    buffer_.data(), buffer_.data() + length, buffered_length_ - length);
  buffered_length_ -= length;
}

std::size_t SerialFrameParser::Feed(
  const std::uint8_t * data, std::size_t length, const FrameCallback & callback)
{
  if (data == nullptr || length == 0) {
    return 0;
  }

  std::size_t consumed_frames = 0;
  for (std::size_t input_index = 0; input_index < length; ++input_index) {
    // Keep the newest bytes if an unbounded noise stream fills the fixed buffer.
    if (buffered_length_ == buffer_.size()) {
      DiscardPrefix(1);
    }
    buffer_[buffered_length_++] = data[input_index];

    while (true) {
      std::size_t sof_index = 0;
      while (sof_index < buffered_length_ && buffer_[sof_index] != io::HEADER_SOF) {
        ++sof_index;
      }
      if (sof_index == buffered_length_) {
        buffered_length_ = 0;
        break;
      }
      if (sof_index > 0) {
        DiscardPrefix(sof_index);
      }

      if (buffered_length_ < sizeof(io::FrameHeader)) {
        break;
      }

      io::FrameHeader header{};
      std::memcpy(&header, buffer_.data(), sizeof(header));
      if (header.data_length < sizeof(std::uint16_t) ||
        header.data_length > kMaxPayloadLength ||
        !Verify_CRC8_Check_Sum(buffer_.data(), sizeof(io::FrameHeader)))
      {
        // Drop only this candidate SOF so a later valid SOF in the same chunk can recover.
        DiscardPrefix(1);
        continue;
      }

      // The command is part of every supported payload and is needed to
      // look up the known payload length and tail expectation per command.
      constexpr std::size_t kCommandOffset = sizeof(io::FrameHeader);
      constexpr std::size_t kCommandEnd = kCommandOffset + sizeof(std::uint16_t);
      if (buffered_length_ < kCommandEnd) {
        break;
      }

      std::uint16_t command = 0;
      std::memcpy(&command, buffer_.data() + kCommandOffset, sizeof(command));
      if (!IsKnownPayloadLength(command, header.data_length)) {
        DiscardPrefix(1);
        continue;
      }

      const std::size_t frame_length = static_cast<std::size_t>(header.data_length) +
        kFrameCrcOverhead +
        (HasFrameTail(command) ? sizeof(io::MsgEndInfo) : 0U);
      if (buffered_length_ < frame_length) {
        break;
      }

      const bool tail_valid = !HasFrameTail(command) ||
        (buffer_[frame_length - sizeof(io::MsgEndInfo)] == io::END1_SOF &&
        buffer_[frame_length - sizeof(io::MsgEndInfo) + 1U] == io::END2_SOF);
      const bool crc16_valid = Verify_CRC16_Check_Sum(
        buffer_.data(), static_cast<std::uint32_t>(header.data_length + kFrameCrcOverhead));
      if (!tail_valid || !crc16_valid) {
        DiscardPrefix(1);
        continue;
      }

      if (callback) {
        callback(buffer_.data(), frame_length);
      }
      ++consumed_frames;
      DiscardPrefix(frame_length);
    }
  }

  return consumed_frames;
}
