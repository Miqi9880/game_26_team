std_msgs/Header header
uint16 id
uint16 mode
float32 yaw             # degree; serial value is forwarded unchanged
float32 yaw_vel         # external unit pending confirmation; forwarded unchanged, not used by core
float32 pitch           # degree; serial value is forwarded unchanged
float32 pitch_vel       # external unit pending confirmation; forwarded unchanged, not used by core
float32 roll            # degree; serial value is forwarded unchanged
float32[4] quaternion   # w, x, y, z
float32 shoot_speed     # m/s
uint16 bullet_count
uint8 game_progress
