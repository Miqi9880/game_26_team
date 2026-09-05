#ifndef ROBOMASTER_PROTOCOL_NEW_H
#define ROBOMASTER_PROTOCOL_NEW_H

#include <cstdint>

// Protocol aligned with game_26/protocol_new.hpp (single canonical wire format).
// Frame: [SOF A5][data_length u16 LE][seq u8][crc8] + [cmd_id u16 LE] + payload
//        + [crc16 u16 LE] + [END 0D 0A]
//   data_length = payload byte count
//   crc8  over header bytes [0..3], stored at [4]
//   crc16 over header+cmd+payload (payload+7 bytes), stored right after payload

#define HEADER_SOF 0xA5
#define END1_SOF   0x0D
#define END2_SOF   0x0A

enum
{
  CHASSIS_ODOM_CMD_ID = 0x0101,
  CHASSIS_CTRL_CMD_ID = 0x0102,  // downlink: RobotCtrlData
  RGB_ID              = 0x0103,
  RC_ID               = 0x0104,
  VISION_ID           = 0x0105   // uplink: VisionData
};

#pragma pack(push, 1)
struct FrameHeader
{
  uint8_t sof = HEADER_SOF;
  uint16_t data_length = 0;
  uint8_t seq = 0;
  uint8_t crc8 = 0;
};

// downlink: vision -> gimbal/electronics (CHASSIS_CTRL 0x0102)
struct RobotCtrlData
{
  float yaw = 0.0f;          // deg, absolute target in shared Z reference
  float yaw_vel = 0.0f;      // deg/s
  float yaw_acc = 0.0f;      // deg/s^2
  float pitch = 0.0f;        // deg, absolute target pitch
  float pitch_vel = 0.0f;    // deg/s
  float pitch_acc = 0.0f;    // deg/s^2
  int8_t target_lock = 50;   // 49=lock(auto aim on) 50=unlock
  int8_t fire_command = 0;   // 0=none 1=fire level (vision decides; safety gate)
};

// uplink: electronics -> vision (VISION 0x0105)
struct VisionData
{
  uint16_t id = 0;
  uint16_t mode = 0;         // 33 = auto aim
  float yaw = 0.0f;          // deg (gimbal/chassis current, shared Z)
  float yaw_vel = 0.0f;      // deg/s
  float pitch = 0.0f;        // deg
  float pitch_vel = 0.0f;    // deg/s
  float roll = 0.0f;         // deg
  float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w,x,y,z
  float shoot_speed = 0.0f;  // m/s
  uint16_t bullet_count = 0;
  uint8_t game_progress = 0;
};
#pragma pack(pop)

static_assert(sizeof(RobotCtrlData) == 26, "RobotCtrlData layout");
static_assert(sizeof(VisionData) == 47, "VisionData layout");

inline constexpr int HEADER_LEN = 5;
inline constexpr int CMD_LEN = 2;
inline constexpr int CRC16_LEN = 2;
inline constexpr int TAIL_LEN = 2;
// total frame bytes for a given payload length
inline constexpr int FRAME_TOTAL(int payload_len) { return HEADER_LEN + CMD_LEN + payload_len + CRC16_LEN + TAIL_LEN; }

#endif  // ROBOMASTER_PROTOCOL_NEW_H