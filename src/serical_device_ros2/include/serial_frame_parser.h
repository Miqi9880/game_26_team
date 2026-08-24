#ifndef ROBOMASTER_SERIAL_FRAME_PARSER_H
#define ROBOMASTER_SERIAL_FRAME_PARSER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>

class SerialFrameParser
{
public:
  static constexpr std::size_t kMaxFrameLength = 512;
  static constexpr std::size_t kFrameOverhead = 11;
  static constexpr std::size_t kMaxPayloadLength = kMaxFrameLength - kFrameOverhead;

  using FrameCallback = std::function<bool(const std::uint8_t *, std::size_t)>;

  // Feed arbitrary chunks. The parser keeps incomplete bytes for the next call.
  // The return value is the number of complete protocol frames consumed.
  std::size_t Feed(const std::uint8_t * data, std::size_t length, const FrameCallback & callback);

  void Reset() noexcept { buffered_length_ = 0; }
  std::size_t buffered_length() const noexcept { return buffered_length_; }

private:
  static bool IsKnownPayloadLength(std::uint16_t command, std::uint16_t payload_length);
  void DiscardPrefix(std::size_t length) noexcept;

  std::array<std::uint8_t, kMaxFrameLength> buffer_{};
  std::size_t buffered_length_{0};
};

#endif  // ROBOMASTER_SERIAL_FRAME_PARSER_H
