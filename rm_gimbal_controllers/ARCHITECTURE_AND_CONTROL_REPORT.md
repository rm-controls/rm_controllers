# rm_gimbal_controllers 架构与控制算法报告

## 摘要

`rm_gimbal_controllers` 采用“双模块协同”设计：

- `Controller`：负责状态机、坐标变换、关节闭环控制与执行器命令输出。
- `BulletSolver`：负责装甲板目标选择、弹道迭代求解、切换装甲过程中的过渡轨迹生成以及射击时序决策。

主要实现入口：

- `src/gimbal_base.cpp`（`Controller::init`、`Controller::update`）
- `src/bullet_solver.cpp`（`BulletSolver` 构造函数、`BulletSolver::solve`）

本报告仅基于当前实现进行说明，不改变运行时行为。

## 1. 架构设计

### 1.1 控制器插件层

`Controller` 以插件形式实现：

- `controller_interface::MultiInterfaceController<rm_control::RobotStateInterface, hardware_interface::ImuSensorInterface, hardware_interface::EffortJointInterface>`

它对应三条硬件/软件集成链路：

- 机器人状态与 TF 查询（`RobotStateInterface`）
- IMU 反馈（`ImuSensorInterface`）
- 力矩命令输出（`EffortJointInterface`）

关键声明位于 `include/rm_gimbal_controllers/gimbal_base.h`。

### 1.2 功能分层

该包可分为决策层、求解层与执行层：

- 决策与模式处理：
  - `Controller` 中的 `rate()`、`track()`、`direct()`、`traj()`
- 命令执行：
  - `Controller` 中的 `moveJoint()`
- 弹道与目标求解子系统：
  - `BulletSolver` 类（`solve`、`getGimbalError`、`judgeShootBeforehand`、`planningPoint`）

### 1.3 运行时数据流

单次更新周期的控制流程如下：

1. 从实时缓冲区读取指令与跟踪数据（`cmd_rt_buffer_`、`track_rt_buffer_`）。
2. 更新变换关系（`odom -> gimbal`、`odom -> base`）并估计底盘速度。
3. 按模式分发（`RATE / TRACK / DIRECT / TRAJ`）。
4. 将目标姿态转换为各轴误差并输入控制回路。
5. 通过关节速度控制器输出最终命令到力矩关节接口。

### 1.4 可观测接口

主要运行时输出包括：

- 云台目标误差话题（`error`）
- 各轴状态话题（`pos_state`）
- 弹道模型可视化标记（`model_desire`、`model_real`）
- 射击时序命令（`shoot_beforehand_cmd`）
- 弹道调试数据（`bullet_solver_data`）

## 2. 控制算法（主链路）

### 2.1 模式状态机

模式切换在 `Controller::update` 中处理：

- `RATE`：对 yaw/pitch 角速度指令积分，更新角度目标。
- `TRACK`：调用弹道解算器，使用解算得到的 yaw/pitch 目标。
- `DIRECT`：对目标点进行几何瞄准（`atan2` 计算 yaw/pitch）。
- `TRAJ`：基于轨迹坐标系姿态叠加偏置命令。

### 2.2 目标生成与关节限位

目标生成流程：

1. 生成云台目标姿态（`setDes`）。
2. 将目标姿态转换为相对底座的 RPY。
3. 按 URDF 关节上下限对各轴命令进行约束（`setDesIntoLimit`）。
4. 发布/记录目标变换（`odom -> gimbal_des`）。

### 2.3 关节闭环控制

关节控制策略由以下部分组成：

- yaw/pitch 位置误差外环（`control_toolbox::Pid`）
- 目标平滑（`NonlinearTrackingDifferentiator`）
- 速度前馈（`yaw_k_v`、`pitch_k_v`）
- IMU 角速度补偿
- 底盘角速度补偿
- pitch 轴可选重力前馈

在 `TRACK` 模式下，yaw 还可使用 `BulletSolver` 提供的轨迹型目标 yaw 与轨迹前馈量。

## 3. 弹道与装甲选择算法

### 3.1 弹道模型

`BulletSolver` 先按弹速区间选择阻力系数，再迭代求解满足命中条件的 yaw/pitch：

- 用含阻力模型估计飞行时间
- 在重力 + 阻力条件下计算弹丸竖直位移
- 用飞行时间更新目标位置
- 迭代直至误差收敛或达到迭代上限

### 3.2 装甲板选择

装甲索引切换依据包括：

- 目标自旋角速度（`v_yaw`）
- 飞行时间后的预测 yaw 偏差
- 切换角阈值与滞回
- 用于切换确认的拟合计数

求解器会更新 `selected_armor_`，并在“跟踪单块装甲”与“中心跟踪”行为间切换。

### 3.3 切装甲过渡轨迹

当满足切换条件时，系统会生成 yaw 三次多项式过渡轨迹：

- 边界条件：起止位置与起止速度
- 求解多项式系数（`a0..a3`）
- 在切换时长内按时间评估轨迹目标
- 在过渡窗口输出轨迹前馈力矩

### 3.4 射击时序决策

`judgeShootBeforehand` 输出以下三种之一：

- `BAN_SHOOT`
- `ALLOW_SHOOT`
- `JUDGE_BY_ERROR`

决策依据为切换时间窗、配置延迟和目标旋转速度。

## 备注

- 本报告是用于架构与算法理解的文档产物。
- 它不会引入任何代码行为变更。
