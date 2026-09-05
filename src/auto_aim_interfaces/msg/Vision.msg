# 上行: 电控 -> 视觉 (对齐 protocol_new.hpp VisionData, cmd_id=0x0105, 500Hz)
std_msgs/Header header
uint16 id
uint16 mode            # 33 = auto aim (自瞄手动切换)
float32 yaw            # [deg] 云台/底盘当前角, 与下行同一零位; +-180 wrap(视觉按旋转矩阵处理)
float32 yaw_vel        # [deg/s]
float32 pitch          # [deg] 云台当前俯仰
float32 pitch_vel      # [deg/s]
float32 roll           # [deg]
float32[4] quaternion  # w, x, y, z
float32 shoot_speed    # [m/s]
uint16 bullet_count    # 电控未填充(=0)
uint8 game_progress    # 电控未填充(=0)