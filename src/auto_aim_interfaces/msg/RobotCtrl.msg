# 下行: 视觉 -> 电控 (对齐 protocol_new.hpp RobotCtrlData, cmd_id=0x0102, <=200Hz)
float32 yaw            # 绝对目标角 [deg], 与上行同一零位 Z; 左转为正; 电控按 error=wrap(cmd-fb) 闭环
float32 yaw_vel        # [deg/s] 前馈 (very_aimer 轨迹速度)
float32 yaw_acc        # [deg/s^2] 前馈
float32 pitch          # 绝对目标俯仰 [deg], 上抬为正; 0 近似水平(偏差用 base_pitch_offset 调)
float32 pitch_vel      # [deg/s] 前馈
float32 pitch_acc      # [deg/s^2] 前馈
int8 target_lock       # 49=锁定(自瞄开) 50=解锁(自瞄关); 由视觉手动切换
int8 fire_command      # 0=不发射 1=允许发射(电平; 由视觉判定命中窗口; 无目标保持0)