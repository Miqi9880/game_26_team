# 外部参考资料笔记

本文件记录本阶段对公开资料的只读查阅结果。外部资料只用于学习检测、几何、跟踪、预测、部署和工程组织思路；它们不是 `game_26_dev` 的协议、单位、坐标系或硬件行为规范。

## 使用边界

- 正式代码和接口以 `/home/ubuntu22/vision-study/game_26_dev` 当前提交、队内确认和 `protocol_new.hpp` 为准。
- 外部资料中的串口帧、CRC、话题名、消息字段、枚举、角度零点、四元数方向、坐标轴和开火时序均不得直接迁入本项目。
- 外部资料中的尺寸、标定数值、模型类别和阈值不能作为 production 配置；没有实测来源时只能放入 `test_only` fixture 或文档示例。
- 本文件中的“可借鉴”是工程假设或验证方向，不是已经完成的比赛性能结论。

## 1. 上海科技大学 2026 视觉技术方案

来源：<https://fcn47qghdcqf.feishu.cn/wiki/Hcw1wxTMZicx0xkinuQcKHetn5d?from=from_copylink>

访问结果：2026-08-24 的只读 HTTP 请求返回 `200`，但最终 URL 重定向到
`https://accounts.feishu.cn/accounts/page/login`。页面正文需要登录，**资料未能访问**；没有把
登录页、搜索摘要或未经核验的转述当作本项目事实，也没有从该页面提取接口、参数或算法结论。

后续若获得公开导出文件或可读链接，应记录实际 URL、查阅日期和引用段落，再仅提炼通用算法/工程
思路，并逐项与本仓库协议、单位、坐标系和安全策略对照。在当前证据下，本节不提供来自该页面的
可执行结论。

## 1.1 用户提供的场景映射（不等同于页面已核验内容）

以下映射由项目负责人在本任务中明确提供，用于把待研究场景对应到当前仓库的模块和验证边界。
它不是从当前不可访问的 Feishu 正文重新推导的参数，也不改变本队的协议、单位、坐标系或安全策略。

| 外部场景 | 当前仓库对应 | 现有证据 | 尚未完成 |
|---|---|---|---|
| 场景 1 模块 2 | OpenVINO / `RawArmorDetection` | `raw_armor_detector.cpp`、`raw_armor_detector_test.cpp`、版本化 model profile 校验 | 正式比赛模型、类别/颜色/四点语义和实测数据 |
| 场景 1 模块 3、场景 4 标定章节 | `PnpStage` | `pnp_stage.cpp`、`pnp_stage_test.cpp`、`docs/pnp_config_schema.md`；test-only 配置显式 opt-in | production K/D、装甲尺寸、camera→gimbal 外参和误差报告 |
| 场景 1 模块 4、场景 5B | `OfflineTracker` / `TargetSelector` | `offline_pipeline.cpp`、`offline_pipeline_test.cpp`；包含时间戳、跳变、短暂/长时间失锁和多目标确定性选择测试 | 正式模型、真实相机数据、实车运动、正式 EKF/预测验收 |
| 场景 1 模块 5、场景 8 | `SafeOfflineAimer` | `offline_pipeline.cpp`、Aimer 测试；相对角诊断和 test-only 绝对零点候选均不开火 | 正式绝对零点、云台控制链路、弹道/延迟和开火联调 |
| 场景 3 | `/Vision_data` 输入适配 | `ros_adapters.hpp`、`auto_aim_core_test.cpp`、串口 loopback；检查时间戳、`frame_id`、四元数 wxyz 和记录字段 | IMU/四元数方向、world 轴、零点和真实联调含义 |
| 场景 9 | Tracker + TargetSelector 状态链 | 多目标关联、检测顺序变化、置信度/中心/历史 track-id 选择测试 | 两车对打、目标切换压力数据、稳定性和实车验收 |

当前能证明的是离线软件边界和安全行为；不能据此宣称正式比赛精度、实车跟踪稳定性、云台闭环或
开火能力已经完成。任何来自 Feishu/论坛的参数、坐标轴、接口或阈值仍须由队内逐项确认后才能进入
production 配置。

本轮审查补充：`auto_aim_pnp_smoke` 和 `auto_aim_offline` 现在必须显式提供版本化
`--model-profile`；test-only 模型和 PnP 配置分别需要显式 opt-in，避免离线链路静默使用未审查
的旧模型语义。`OfflineTracker` 对负数、回退和重复时间戳返回安全快照时，会同步报告
`TempLost` 状态；这只是诊断一致性修复，不代表增加了正式预测或多目标稳定性能力。

## 2. RoboMaster 论坛

来源：<https://bbs.robomaster.com>

访问结果：2026-08-24 论坛首页返回 HTTP 200。当前没有在本阶段指定或核验某一篇具体技术帖，因此本文件不把论坛首页当作算法参数来源，也不引用无法复核的帖子结论。

### 可借鉴的使用方式

- 后续若队员指定具体帖子，应记录帖子 URL、标题、作者/发布时间（若页面提供）、查阅日期和实际引用段落。
- 对检测、PnP、EKF、弹道或部署经验，只抽取可迁移的工程问题清单，例如时间戳、坐标变换、延迟、参数版本和日志证据。
- 任何论坛代码、协议字段、阈值、坐标约定和硬件时序都必须先与队内确认逐项对照；未确认的内容只能作为待验证假设。

## 3. 队伍 fork 与官方上游页面

来源：

- <https://github.com/Miqi9880/game_26>
- <https://github.com/HJ-vision/game_26>

访问结果：2026-08-24 两个仓库页面均返回 HTTP 200。GitHub 页面只用于确认公开仓库可访问和查看公开工程上下文；本地唯一正式工作区仍是 `/home/ubuntu22/vision-study/game_26_dev`。

本地开发约定：

- `origin` 指向 `https://github.com/Miqi9880/game_26.git`；`upstream` 指向 `https://github.com/HJ-vision/game_26.git`。
- 不从网页分支或旧副本直接覆盖本地未提交成果；需要同步时先检查差异、协议和测试。
- 上游/队伍页面中的旧 `protocol.h`、旧消息定义或旧单位不能替换当前 `protocol_new.hpp` 和 ROS 接口。

## 4. 与当前实现的对应关系

以下是本地代码的现状审计与后续研究方向对应关系，不表示上科大页面或论坛首页已提供可直接引用的
实现细节；当前可核验的外部页面只证明了访问状态。

| 参考思路 | 本地承接位置 | 当前状态/限制 |
|---|---|---|
| 四点检测与预处理逆变换 | `src/auto_aim_ros2/src/raw_armor_detector.cpp` | 已有显式 letterbox 和点序测试；正式比赛模型仍待提供和核验 |
| `solvePnP` 与几何校验 | `src/auto_aim_ros2/src/pnp_stage.cpp`、`docs/pnp_config_schema.md` | test-only 合成配置可验证；无 production 标定数据 |
| 观测跟踪与目标选择 | `src/auto_aim_ros2/src/offline_pipeline.cpp` | 已有超时、时间戳、跳变和失锁测试；不能据此宣称实车性能 |
| ROS 2 日志/离线回放 | `src/auto_aim_ros2/src/ros_backend.cpp`、`docs/offline_pipeline_design.md` | `null`、`mock`、`offline_reference` 均受 dry-run 安全门控 |
| 相机与 Orin 部署 | `docs/orin_real_camera_bringup.md` | 仅准备清单；尚未在 Orin 编译或连接真实相机 |

## 5. 后续验证问题

1. 获取正式比赛模型后，核对输入尺寸、颜色顺序、预处理、输出 shape、类别/颜色/大小语义和四点顺序，并加入版本化 profile。
2. 使用比赛相机和实际分辨率完成 K/D、装甲尺寸、camera→gimbal 外参和误差报告；在此之前继续拒绝 production PnP 配置。
3. 用带时间戳的离线帧验证跟踪、延迟补偿和预测；所有调参结果附带 commit、模型、标定和数据集版本。
4. 在电控确认速度/加速度单位、绝对零点、四元数方向、golden frame、watchdog 和开火语义前，保持外部速度/加速度为 0，`fire_command=0`。
