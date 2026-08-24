#include "serial_frame_parser.h"

#include <cstring>

#include "crc.h"
#include "protocol_new.hpp"

bool SerialFrameParser::IsKnownPayloadLength(
  std::uint16_t command, std::uint16_t payload_length)
{
  switch (command) {
    case io::VISION_ID:
      return payload_length == sizeof(io::VisionData);
    case io::CHASSIS_CTRL_CMD_ID:
      return payload_length == sizeof(io::RobotCtrlData);
    default:
      return false;
  }
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
      if (header.data_length > kMaxPayloadLength ||
        !Verify_CRC8_Check_Sum(buffer_.data(), sizeof(io::FrameHeader)))
      {
        // Drop only this candidate SOF so a later valid SOF in the same chunk can recover.
        DiscardPrefix(1);
        continue;
      }

      const std::size_t frame_length =
        static_cast<std::size_t>(header.data_length) + kFrameOverhead;
      if (buffered_length_ < frame_length) {
        break;
      }

      const std::size_t tail_offset = frame_length - sizeof(io::MsgEndInfo);
      const bool tail_valid =
        buffer_[tail_offset] == io::END1_SOF && buffer_[tail_offset + 1] == io::END2_SOF;
      const bool crc16_valid = Verify_CRC16_Check_Sum(
        buffer_.data(), static_cast<std::uint32_t>(header.data_length + 9));

      std::uint16_t command = 0;
      std::memcpy(&command, buffer_.data() + sizeof(io::FrameHeader), sizeof(command));
      if (!tail_valid || !crc16_valid || !IsKnownPayloadLength(command, header.data_length)) {
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
