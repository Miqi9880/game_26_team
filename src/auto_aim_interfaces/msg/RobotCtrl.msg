float32 yaw             # degree absolute target angle; hold last valid value when input is stale
float32 yaw_vel         # external unit pending confirmation; currently always 0
float32 yaw_acc         # external unit pending confirmation; currently always 0
float32 pitch           # degree absolute target angle; hold last valid value when input is stale
float32 pitch_vel       # external unit pending confirmation; currently always 0
float32 pitch_acc       # external unit pending confirmation; currently always 0
int8 target_lock        # 49=locked, 50=unlocked
int8 fire_command       # 0=none, 1=burst, 2=single
