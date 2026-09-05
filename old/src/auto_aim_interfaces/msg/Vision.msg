std_msgs/Header header
uint16 id
uint16 mode
float32 yaw             # degree; serial value is forwarded unchanged
float32 yaw_vel         # degree/s; forwarded unchanged, diagnostic-only in current core
float32 pitch           # degree; serial value is forwarded unchanged
float32 pitch_vel       # degree/s; forwarded unchanged, diagnostic-only in current core
float32 roll            # degree; serial value is forwarded unchanged
float32[4] quaternion   # w, x, y, z; relative to power-on origin, direction not interpreted
float32 shoot_speed     # m/s
uint16 bullet_count
uint8 game_progress
