float32 yaw             # degree absolute target in shared power-on reference; hold on timeout
float32 yaw_vel         # degree/s; safety output currently always 0 pending MCU feedforward semantics
float32 yaw_acc         # degree/s^2; safety output currently always 0 pending MCU feedforward semantics
float32 pitch           # degree absolute target in shared power-on reference; hold on timeout
float32 pitch_vel       # degree/s; safety output currently always 0 pending MCU feedforward semantics
float32 pitch_acc       # degree/s^2; safety output currently always 0 pending MCU feedforward semantics
int8 target_lock        # 49=locked, 50=unlocked
int8 fire_command       # 0=none, 1=burst, 2=single
