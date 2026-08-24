# 图像与 Vision 状态时间对齐边界

本文档记录 `/image_raw` 与 `/Vision_data` 在当前仓库中的时间戳边界。当前实现只做数据完整性检查和诊断，不把配对结果接入 Tracker、Aimer、`/Robot_ctrl_data` 或开火链路。

## 时间戳来源

当前链路中存在三个必须分开记录的时间：

1. 图像 Header 时间：`sensor_msgs/msg/Image.header.stamp`，由相机驱动或上游 ROS 发布者写入，进入 `ImageFrame.stamp_ns`。
2. Vision Header 时间：`auto_aim_interfaces/msg/Vision.header.stamp`，由 `VisionPub` 在串口接收后使用 `this->now()` 写入；它不是 `VisionData` 串口结构体中的 MCU 原始时间字段。
3. 本机 steady-clock 接收时间：节点收到图像或 Vision 回调时调用 `std::chrono::steady_clock::now()` 记录，仅表示本机回调观测时间。

当前没有证据证明图像 Header 和 Vision Header 使用同一时钟域，也没有证据证明任一 Header 与本机 steady clock 或 MCU 时钟具有可比的零点、漂移和延迟关系。因此：

- Header 时间不被假定为 MCU/IMU 原始时间；
- steady-clock 接收时间不能与 ROS Header 纳秒值直接相减；
- ROS 节点默认把 Vision 样本标为 `VisionHeader`、图像标为 `ImageHeader`，配对结果为 `incomparable`，直到队内确认共享时钟契约；
- 日志同时保留两类 Header 值和两类本机接收时间，避免把回调到达时间伪装成传感器采样时间。

## 有界历史与配对状态

`vision_time_alignment::VisionStateHistory` 是纯 C++、固定容量的诊断组件：

- 按时间戳严格递增接收样本，拒绝零/负时间戳、回退、重复和未知/不匹配时间域；
- 检查角度、射速和四元数元素是否有限，并拒绝零范数四元数；四元数数组只按已确认的 wxyz 保存，不解释方向或做归一化；
- 满容量时淘汰最旧样本，不无限增长；
- `PairConfig.tolerance_ns` 使用显式可选值。未配置时返回 `unconfigured`，只产生诊断；
- 默认不允许未来 Vision 样本配对到图像；只有显式开启 `allow_future` 才改变这一诊断策略；
- `matched` 之外的结果不暴露样本，形成 fail-closed 边界。

配对状态包括 `matched`、`missing`、`stale`、`future`、`invalid`、`incomparable` 和 `unconfigured`。组件不做坐标变换、角度单位转换、姿态积分、IMU 解算、运动补偿或延迟预测。

ROS 参数默认值为：

```text
vision_history_capacity = 32
vision_time_alignment_tolerance_ns = -1   # 未配置
vision_time_alignment_allow_future = false
vision_time_alignment_assume_shared_ros_clock = false
```

ROS 适配层只把状态插入历史并记录配对诊断。Vision 的 yaw/pitch/roll、四元数和时间差不会改变现有控制命令；dry-run 仍强制 `serial_enabled=false`、`allow_fire=false` 和 `fire_command=0`。既有 offline CSV schema 不增加字段，配对信息通过日志输出。

只有在联调证据已经确认两条 Header 使用同一时钟域时，才可以显式设置
`vision_time_alignment_assume_shared_ros_clock=true` 并提供非负
`vision_time_alignment_tolerance_ns`。这个参数是队内已确认契约的声明，不是节点自行证明同步；默认值保持 `false`，因此图像和 Vision Header 仍标记为不可比较。

## 当前可信边界

本阶段可证明的是：非法样本不会进入时间历史；历史容量有界；回退/重复/跨时钟域/过期/未来/未配置容差会得到明确诊断；ROS dry-run 的配对成功或失败都不会产生开火命令。

本阶段不能证明：图像与 Vision 的真实采样时刻相同、串口延迟固定、Vision Header 代表 MCU 采样时刻、四元数可用于世界坐标变换、或配对结果足以支持云台预测。`matched` 只在同一已声明时间域的纯组件测试中成立，不是对当前真实 ROS 两条 Header 时间线已经同步的声明。

## 后续真实联调必须确认

在任何生产接入前，队内需要用实测记录确认：

- IMU 四元数的方向（IMU→world 还是 world→IMU）、轴定义、零点和右手系；
- 图像 Header、Vision Header、MCU/IMU 时间戳之间的时钟关系、同步方式、漂移和回绕；
- 串口发送、接收和 ROS 调度延迟，以及是否需要硬件序号或 golden frame；
- 相机曝光开始/结束时间与发布 Header 的定义；
- MCU 时间基准、上电零点、重启行为和跨设备时间换算；
- 关联容差、未来样本策略、丢帧/回退处理和 watchdog 规则。

确认前不得把 Vision 状态接入正式绝对角、Tracker 预测、云台闭环或 `fire_command`。

## 与参考资料的关系

这项工作对应队内参考资料中的“场景 3”（IMU、四元数和时间戳联调）。相关飞书正文当前需要登录，无法作为可核验参数来源；因此本文只记录当前仓库已证实的 ROS/串口边界，不把飞书文档中的参数、坐标系或接口写入本队正式协议。

## 验证与回退

纯组件测试覆盖精确配对、无状态、过期、未来、回退、重复、非有限四元数、容量淘汰和跨时间域不可比较。ROS topic-level dry-run 覆盖 Vision 输入与图像输入同时存在、配对诊断不改变 mock 控制角、非法图像时间戳，以及所有情况下 `fire_command=0`。

若真实联调发现时间域或语义不可靠，保持 `vision_time_alignment_tolerance_ns=-1`、`vision_time_alignment_allow_future=false`，或按提交粒度 revert 本分支提交；不得通过猜测时钟关系来“修正”结果。
