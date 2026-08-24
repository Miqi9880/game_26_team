# RoboMaster 视觉自瞄项目协作规则

本文档是 game_26_dev 的团队协作约定，适用于代码、ROS 2 接口、串口协议、检测与 PnP、离线自瞄、标定配置、实验记录和答辩材料。目标是让改动可审查、可复现、可回退，并把离线验证与真实硬件风险分开。

如果本文件与用户最新明确指令、队内正式协议或主办方新规则冲突，以最新且明确的要求为准；执行时在任务记录或 PR 中写明冲突和依据。未经明确授权，不因本文件而自动 commit、push、merge 或创建 PR。

## 1. 项目范围和仓库结构

正式开发仓库只有：

    /home/ubuntu22/vision-study/game_26_dev

旧 Windows 副本、旧正式仓库和旧规则实验仓库只能作为只读参考，不能作为正式代码来源。提交前必须确认位于上述仓库。

主要目录和文件：

- protocol_new.hpp：当前正式串口结构体、帧常量和协议布局；旧 protocol.h 只保留参考。
- src/auto_aim_interfaces：ROS 2 Vision.msg 和 RobotCtrl.msg 接口包。
- src/serical_device_ros2：串口设备、帧解析、CRC、ROS 串口桥和控制安全策略。
- src/auto_aim_ros2：图像适配、OpenVINO 检测、PnP、离线 Tracker/Selector/Aimer、ROS backend 和测试工具。
- src/ros2-hik-camera：海康相机 ROS 2 节点、启动文件和相机配置；硬件依赖需单独核验。
- docs：接口、单位、标定、离线流程、Tracker 和 Orin/相机联调文档。
- 视觉校内赛规则.md：校内赛规则资料。
- build/、install/、log/：本地构建和测试产物，不得提交。

## 2. 团队角色和协作方式

每个任务应明确负责人和审查人：

- 视觉算法负责人：检测、Armor、PnP、Tracker、TargetSelector、Aimer 和离线实验。
- ROS/系统负责人：消息接口、节点、QoS、图像适配、launch 和集成测试。
- 串口/电控接口负责人：protocol_new.hpp、CRC、帧解析、字段映射和 golden frame。
- 硬件/标定负责人：相机、内参、装甲尺寸、camera→gimbal 外参、Orin 和实车记录。
- 文档/答辩负责人：设计记录、实验数据、风险清单和答辩材料。

负责人提出最小变更和验证证据，审查人检查接口、单位、安全和回退路径。跨模块改动须点名相关负责人；未确认的问题记录为待确认项，不用猜测补齐。

## 3. 分支命名规则

main 是稳定基线，不直接提交功能开发代码。每个独立任务使用功能分支，例如：

    feat/pnp-stage
    feat/tracker
    feat/aimer
    feat/serial-parser
    fix/timeout-safety
    test/offline-pipeline
    docs/calibration

分支名使用小写并表达单一任务边界。发现分支基线过旧时，先与负责人确认同步方式，不使用破坏性 Git 命令覆盖他人修改。

## 4. commit 规则

- 每次修改前先运行 git status --short，确认工作区和他人改动。
- commit 应小而独立，信息清楚说明实际改动。
- 一个 commit 不混合无关功能、全仓格式化、临时调试输出或生成物。
- 推荐格式：

      feat: add offline PnP stage
      fix: reject invalid serial frame
      test: add tracker timeout cases
      docs: record calibration units

- 前缀可用 feat、fix、test、docs、refactor；正文必要时记录原因、接口影响、测试和限制。
- 未经用户或项目负责人明确授权，不得 commit；授权后也不能使用未经审查的 git add .。

## 5. push 规则

- origin 应指向队伍 fork，upstream 应指向官方仓库；push 前执行 git remote -v 核对。
- 不得直接 push 到 main。功能分支必须通过 PR 合并到 main。
- push 前先运行相关测试，检查 git diff --check、暂存文件和远程目标。
- 未经用户明确授权不得 push；不得 force push，不得把 token、密码或凭据写入 URL、脚本或仓库。
- 每次允许 push 前，先报告目标远程/分支、commit、文件和功能摘要、测试结果、已知风险。
- push 成功后报告实际 commit SHA、远程分支、GitHub 链接、推送内容、测试结果和是否创建 PR。

## 6. Pull Request 规则

push 功能分支后，通过 PR 合并到 main。未经明确授权不得创建或 merge PR，合并权限由项目负责人保留。

PR 必须说明：

- 改动目的和问题背景；
- 主要文件和模块；
- 测试命令、完整结果和未运行的测试；
- 已知限制、失败场景和回退方法；
- 是否涉及串口、坐标系、单位、标定、模型或硬件；
- 是否需要真实相机、Orin、云台或实车验证；
- 协议或数据来源的确认人/记录位置。

## 7. 代码审查清单

PR 合并前检查：

- [ ] 改动范围单一且清楚；
- [ ] 没有误改协议字段、帧长度、CRC、字节序或命令号；
- [ ] 单位和坐标系说明完整，变量名与单位一致；
- [ ] 新增功能已有对应测试；
- [ ] 旧测试没有回归，失败没有被忽略或伪装成功；
- [ ] dry_run 和离线测试仍不开火；
- [ ] serial_enabled 默认安全值没有被放宽；
- [ ] 没有提交临时文件、模型缓存、设备文件或真实隐私数据；
- [ ] 已说明实车风险、未验证项和回退方法；
- [ ] 配置和文档已同步；
- [ ] 必要时已请求视觉、ROS、串口/电控和硬件队友审查。

## 8. 构建和测试要求

每次修改前先检查状态；每次代码、配置或接口修改后至少在以下目录执行：

    cd /home/ubuntu22/vision-study/game_26_dev
    source /opt/ros/humble/setup.bash

    colcon build --symlink-install \
      --packages-select auto_aim_interfaces serical_device_ros2 auto_aim_ros2

    source install/setup.bash

    colcon test \
      --packages-select auto_aim_ros2 serical_device_ros2 \
      --event-handlers console_direct+

    colcon test-result --verbose
    git diff --check

构建或测试失败必须如实报告，先修复或记录阻塞原因。新功能必须增加测试，至少覆盖正常路径、无效输入、超时/失锁和安全输出。backend=null、backend=mock 或显式配置的 backend=offline_reference 只能用于 dry-run；离线回放不代替真实硬件验证。

日志、CSV、模型输出和临时标注图默认写入 /tmp 或被 .gitignore 排除，除非明确作为可复现测试样例纳入并说明来源。

## 9. ROS 2、串口和协议修改规则

正式串口结构体必须使用 protocol_new.hpp。通信方向：

    下位机 VisionData → VisionPub → /Vision_data → 视觉程序
    视觉程序 → /Robot_ctrl_data → RobotCtrlSub → 下位机 RobotCtrlData

- VisionData 是下位机到上位机状态输入；RobotCtrlData 是上位机到下位机控制输出。
- /Vision_data 类型为 auto_aim_interfaces/msg/Vision。
- /Robot_ctrl_data 类型为 auto_aim_interfaces/msg/RobotCtrl。
- /Vision_data 是下位机状态输入；/Robot_ctrl_data 是视觉控制输出。
- 串口映射层只做字段映射，不做角度单位转换。
- 外部位置角 yaw/pitch/roll 使用 degree。
- 算法内部角度使用 rad；内部速度/加速度使用 rad/s、rad/s²。
- 速度和加速度外部单位未确认前，ROS/串口输出保持 0。
- 单位链路固定为：VisionData degree → Vision.msg degree → 输入适配 degree→rad → 算法/PnP/Aimer rad → 输出适配 rad→degree → RobotCtrl.msg degree → RobotCtrlData degree。
- PnP 的 relative_yaw_rad、relative_pitch_rad 是相对几何量，不能直接当作 RobotCtrl 绝对目标角。
- 没有绝对零点或正式外参时，不得把相对角伪装成正式控制角；test_absolute_zero 必须带 test_only 标记。
- fire_command 在 dry-run 和离线测试中必须为 0。
- 未确认的零点、坐标系、四元数方向、端序、float/packed ABI、golden frame、watchdog、ACK、断线行为和开火时序不得自行猜测。

修改协议时必须同时更新结构体、收发解析、ROS 映射、静态布局检查、loopback/坏帧测试和接口文档；只改头文件而不验证实际收发不合格。

## 10. 硬件安全规则

默认配置必须为：

    serial_enabled=false
    dry_run=true
    allow_fire=false
    fire_command=0

- 离线测试不连接真实串口、真实相机、云台或发射机构。
- 坐标、单位、零点和协议联调未完成前，不允许云台闭环运动。
- 未确认 fire_command=1/2 的脉冲/电平语义、保持时间和停止规则前，不允许发送 1 或 2。
- 失锁、超时、时间戳异常、NaN/Inf、非法帧、无效标定或异常必须进入安全输出：保持最近有效位置（若适用）、清零速度/加速度、解锁并不开火。
- 真实硬件测试须记录日期、设备/固件、代码 commit、配置/标定版本、人员、停止条件和回退方式。
- 不得把 test_only 标定数据、测试零点或旧参考模型结果用于实车控制。

## 11. 配置、标定和实验数据规则

- production 配置和 test_only 配置必须分开存放、命名和记录来源；默认不得加载假标定。
- 相机标定记录 K、D、图像宽高、标定板规格、重投影误差、日期、版本、缩放/letterbox 和坐标系。
- 装甲配置记录 small/big 实际宽高（单位 m）、3D 角点顺序、class_id/color_id/type 映射和模型版本；未知类别必须拒绝。
- 外参记录 R_gimbal_from_camera、t_gimbal_from_camera（单位 m）、源/目标坐标系、来源、日期和版本；未配置时禁止假设 identity 或同轴。
- 新模型记录输入/输出 shape、颜色顺序、预处理、类别/颜色语义、small/big 映射、四点顺序和版本。
- 实验数据带 commit、配置/标定版本、模型版本、设备、时间、帧率和单位；输出不能脱离元数据流传。
- 未验证结果不得写成正式性能、距离、角度、命中率或延迟结论；测试样例不能冒充比赛模型或生产标定。

## 12. 文档和答辩要求

接口、单位、坐标、标定来源、测试命令、日志摘要和限制必须与代码同步。答辩材料应能从提交 SHA 追溯模型、配置、标定、ROS/串口方向、degree/rad 边界、失败保护、不开火策略以及离线证据和实车证据的分界。

不得用没有来源的截图、手工改写 CSV 或未记录配置的曲线作为正式答辩证据。

## 13. 问题报告和变更记录

问题报告至少包含：复现步骤、执行目录、代码 commit、ROS 参数、模型/标定版本、输入文件或话题、实际/预期输出、日志、严重程度和硬件影响。安全、协议、单位或坐标问题须标高优先级并通知对应负责人。

重要变更记录原因、文件、接口影响、测试结果、限制、回退方法、待确认人和后续任务。实验报告不能只写“通过”，必须写明通过范围和未验证范围。

## 14. 回退和故障处理规则

- 优先通过保留提交历史、关闭/修正功能分支或恢复到稳定 commit 回退；不得使用 git reset、git checkout -- 或 git clean，也不得删除、覆盖或回退既有未提交修改。
- 回退前保存日志、测试结果和当前 SHA，确认目标范围，必要时请负责人批准。
- 构建失败时保留失败日志，检查依赖、环境和变更边界；不要为了“变绿”删除测试或放宽安全检查。
- 运行时异常优先切换到 backend=null、dry_run=true、serial_enabled=false、fire_command=0，停止真实硬件并记录时间。
- 串口帧、标定、坐标或开火语义不确定时，停止扩大改动，标记待确认项并保留可回退版本。
- 修复完成后重新运行构建、全量测试和 git diff --check，记录故障原因、修复 commit 和验证证据。

## 开发窗口结束检查

每个任务结束时报告：改动文件、执行目录、验证命令和结果、commit/push/PR 是否发生、已知风险和回退方法，并明确说明本次是否遵守本文件。若用户最新明确指令与本文件冲突，以用户指令为准并在报告中记录冲突和处理方式。本文件的创建任务只新增文档，不自动 commit、push 或创建 PR；后续每次开发任务开始前应先阅读本文件并先检查 git status。
