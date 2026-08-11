# Apple A3 安全纯规划入口设计

**日期：** 2026-07-14

**状态：** 已批准并实施；两次正式 Apple A3 plan-only 均通过。A3 已完成，
Phase B 仍受 `>=18 Hz` 真实运动健康门约束

**目标：** 为 Apple Phase A3 提供一个独立、可重复、默认不执行运动的
`grasp_plan_only` 命令。该命令复用现有 perception、round-top selector、
真实场景 AABB、approach corridor 和 MoveIt planner，只验证候选的几何安全性
与 pregrasp 可达性；绝不执行轨迹、Cartesian servo 或夹爪命令。

## 1. 验收门调整

`>=18 Hz` 不再是 Apple A3 plan-only 的前置条件。A3 使用功能性场景门：

1. 新进程必须从 reliable/volatile `/scene_clearance_bounds` 收到一条新消息；
2. 该消息必须原子包含当前 6 个场景物体；
3. marker center/scale 必须有限且 scale 严格为正；
4. round-top 在缺失、无效或超时场景下继续 fail closed；
5. A3 将场景等待时间设为 `1.5 s`，适应当前约 `3.43 Hz` 的单路相机环境及
   偶发软件渲染停顿；
6. 连续 2 次独立 reset + fresh perception + plan-only 均无场景超时并得到
   至少一个可达候选。

`18-20 Hz` 只保留为真正运动前对 control loop 和 `/joint_states` 的健康门，
不再要求 scene bounds/description 在静态规划阶段持续达到该频率。

## 2. 当前入口为何不能用于 A3

- `GRASP_DEBUG_STOP_AT_PREGRASP=1` 会先真实执行 `move_to_pre_grasp`，属于
  Phase B，不是 A3。
- `grasp_demo` 总会进入 `execute_first_reachable_plan()`。即使 MoveIt server
  开启 plan-only，后续 Cartesian 和 gripper steps 仍可能执行。
- 手工先调用 `set_planonly` 再运行 `grasp_demo` 不是安全方案。

因此 A3 必须使用独立命令，不能依赖现有 debug stop。

## 3. 方案选择

### 方案 A：独立 `grasp_plan_only` 命令（采用）

创建只含 planning client 的 ROS 节点，显式开启 arm_api2 plan-only，逐候选
验证 pregrasp MoveIt 可达性，并在 `finally` 中关闭 plan-only。

优点：入口清晰、默认安全、可测试、可重复。改动限制在一个新模块、setup
entrypoint 和一个聚焦测试文件。

### 方案 B：在 `grasp_demo` 增加环境变量分支（不采用）

改动较少，但规划和执行共享入口；任一条件判断回归都可能落入 Cartesian 或
gripper step，安全边界不清晰。

### 方案 C：手工 service + 临时 Python 命令（不采用）

无需正式入口，但难以保证清理 plan-only 状态、候选 fallback 和重复试验流程
一致，也无法建立稳定回归测试。

## 4. 运行架构

新增 `my_course_pkg.grasp.plan_only`：

1. 创建 `GraspPlanOnlyNode`，只实例化 `ArmApi2Client`；不创建
   `GripperController` 或 `ArmMotionExecutor`。
2. 在 executor 已于后台 spin 的 worker 中读取 `/arm/state/current_pose`，转换为
   planner 所需的 6D start pose。
3. 调用 `plan_pick_place_candidates_from_perception()`：
   - 读取 fresh FoundationPose 结果；
   - 执行 round-top selector；
   - 加载 `/scene_clearance_bounds`；
   - 对完整 pregrasp -> grasp 线段执行 corridor 过滤；
   - 返回通过几何门的排序候选。
4. 调用 `ArmApi2Client.set_planonly(True)`；失败则停止，不发送 action goal。
5. 切换到 `CART_TRAJ_CTL`，这是 MoveIt action server 接受 pose goal 的前置状态；
   该切换本身不发送运动目标。
6. 依次把每个候选的 `pre_grasp_pose_6d` 发送到
   `ArmApi2Client.move_to_pose()`。MoveIt server 在 plan-only 模式下只调用
   planner，不调用 `execute(plan)`。
7. 第一个成功候选即为 A3 结果；失败候选记录原因并继续下一个。
8. 所有候选失败时返回非零并报告结构化错误。
9. `finally` 始终调用 `set_planonly(False)`。若恢复失败，命令也返回失败并明确
   提示后续 motion 继续阻塞。

## 5. final approach 的验证边界

现有真实执行计划只有 `move_to_pre_grasp` 使用 MoveIt；pregrasp -> grasp 使用
Cartesian servo。A3 因此采用与实际架构一致的验证：

- MoveIt plan-only 验证当前姿态 -> pregrasp；
- 已实现的 round-top corridor 检查验证 pregrasp -> grasp 全直线段与所有
  非目标物体的距离；
- planner 现有桌面、world-Z 和候选几何门继续生效；
- A3 不伪称 MoveIt 已规划 Cartesian final approach。

对 final approach 做 MoveIt CartesianPath plan-only 属于未来增强，不是本次
A3 的必要条件。

## 6. 安全不变量

`grasp_plan_only` 必须满足：

- 不导入或实例化 `GripperController`；
- 不实例化 `ArmMotionExecutor`；
- 不调用 `execute_plan()` 或 `execute_first_reachable_plan()`；
- 不调用 `send_pose_cmd()`、`interpolate_to_pose()` 或 FollowJointTrajectory；
- 不发送任何 gripper action/topic；
- 不调用 `return_to_initial_pose()`；
- 不使用 `GRASP_DEBUG_STOP_AT_PREGRASP`；
- plan-only 状态的开启与恢复均有日志和失败传播。

## 7. 实施文件

运行代码只修改 3 个文件：

1. 新增 `src/my_course_pkg/my_course_pkg/grasp/plan_only.py`；
2. 修改 `src/my_course_pkg/setup.py`，注册 `grasp_plan_only`；
3. 新增 `src/my_course_pkg/test/test_grasp_plan_only.py`。

实施完成后更新路线图与 handoff，但不借机重构 executor、gripper 或相机线程。

## 8. 测试

聚焦测试使用 fake node/client/planning result，至少覆盖：

1. `set_planonly(True)` 成功后才发送 planning goal；
2. 第一个候选成功时停止；
3. 第一个失败时尝试第二个；
4. 全部失败时返回明确错误；
5. planner/MoveIt 异常时仍调用 `set_planonly(False)`；
6. 开启 plan-only 失败时不发送 goal；
7. 恢复 plan-only 失败时命令失败；
8. 不存在 executor、Cartesian servo 或 gripper 调用路径；
9. package console entrypoint 指向正确函数。

随后运行现有 planner、trajectory、executor 和 simulator clearance 聚焦回归，
以及 `colcon build --packages-select my_course_pkg --symlink-install`。

## 9. Live A3 两次流程

每次试验：

1. reset simulation；
2. 运行 fresh `pipeline`，指令为 `pick up the apple`；
3. 确认自动选择 Apple mask，且不设置 `FOUNDATIONPOSE_MASK_INDEX`；
4. 设置 `GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC=1.5`；
5. 运行 `ros2 run my_course_pkg grasp_plan_only`；
6. 保存候选数量、corridor 过滤、MoveIt plan-only 结果、选中候选和完整日志；
7. 读取 robot/object pose，确认相较试验前没有机械臂、夹爪或物体运动。

两次均满足以下条件才通过 A3：

- fresh scene bounds 完整加载；
- 至少一个 round-top 候选通过 corridor；
- 至少一个候选的 pregrasp MoveIt plan-only 成功；
- MoveIt 日志显示 plan-only，未出现 execution；
- robot、gripper 和 Apple 位姿未因 A3 命令改变。

A3 通过后仍不能直接抓取；下一阶段要求两次独立
`GRASP_DEBUG_STOP_AT_PREGRASP=1` 的 Phase B 运动验证，且开始 Phase B 前必须先
通过 `>=18 Hz` control loop 与 `/joint_states` 健康门。
