# MoveIt 抓取规划使用指南

这份文档解释一件事：

```text
FoundationPose + grasp 数据库给出“要抓哪里”
MoveIt 负责判断“机械臂能不能安全到那里，以及怎么过去”
```

## 1. 先分清两个规划

抓取任务里其实有两层规划。

第一层是 grasp planning：

```text
物体在哪里？
夹爪应该从哪个方向接近？
最终抓取位姿是什么？
```

它的输入通常是：

```text
T_world_object
T_object_grasp candidates
```

输出是：

```text
grasp_pose_6d
pre_grasp_pose_6d
```

第二层是 motion planning：

```text
机械臂当前在这里，目标末端位姿在那里
关节怎么动？
会不会碰撞？
IK 能不能解出来？
路径是否可执行？
```

这部分才是 MoveIt 的工作。

所以：

```text
grasp 数据库负责生成候选抓取姿态
MoveIt 负责验证和执行这些候选姿态
```

## 2. 5000 个 grasp pose 和 MoveIt 的关系

你说的 5000 个 `.npy/.npz` grasp pose，通常是某个物体的候选抓取姿态。

它们大概率表示：

```text
T_object_grasp
```

也就是：

```text
夹爪在物体坐标系下的候选姿态
```

这些候选本身不是 MoveIt 规划结果。

它们只是告诉你：

```text
这个物体可以从这些方向抓
```

真正给机械臂执行前，需要把它变成 world/base 下的末端目标：

```text
T_world_grasp = T_world_object @ T_object_grasp
```

然后再交给 MoveIt：

```text
MoveIt 检查这个 T_world_grasp 是否可达
MoveIt 规划从当前机械臂状态到这个 pose 的路径
```

所以关系是：

```text
5000 个 grasp pose
 -> 生成 5000 个候选 T_world_grasp
 -> MoveIt 挨个尝试 IK / planning
 -> 选一个可达且路径安全的，不一定是 top-down
```

## 3. 当前代码里哪些部分属于 grasp planning

当前这些文件属于抓取姿态规划：

```text
grasp_selector.py
trajectory_planner.py
pick_place_planner.py
```

尤其是：

```python
T_world_grasp = T_world_obj @ T_obj_grasp
```

这个步骤就是把 grasp 数据库里的候选姿态转成机器人世界坐标系下的目标末端位姿。

当前代码里：

```python
select_preferred_grasp(...)
```

只是做了一个很简单的排序规则：

```text
优先选择工具 +Z 轴更接近世界 -Z 的抓取
```

也就是默认偏好从上往下抓，但这不是硬规则。

真实抓取不一定非要从上方抓。比如：

```text
细长物体可能更适合侧抓
有把手的物体可能更适合从特定方向抓
物体上方有遮挡时需要侧向接近
从上方抓可能 IK 不可达或会碰撞
```

它还没有做：

```text
MoveIt IK 检查
MoveIt 碰撞检查
MoveIt 路径规划
```

## 4. MoveIt 应该接在哪里

最推荐的位置是 `grasp_selector.py` 里。

当前逻辑大概是：

```text
读取所有 grasp candidates
按简单规则选一个 preferred grasp
返回 grasp_pose_6d
```

更完整的逻辑应该是：

```text
读取所有 grasp candidates
按规则排序，比如可达性、碰撞风险、离当前末端距离、接近方向是否适合当前场景
对每个 candidate：
  1. 算 T_world_grasp
  2. 算 pre_grasp_pose
  3. 问 MoveIt：pre_grasp 是否 IK 可达？
  4. 问 MoveIt：grasp 是否 IK 可达？
  5. 问 MoveIt：从当前状态到 pre_grasp 是否能规划？
  6. 如果都成功，就选择它
```

伪代码：

```python
for T_obj_grasp in grasp_candidates:
    T_world_grasp = T_world_obj @ T_obj_grasp
    grasp_pose_6d = matrix_to_pose6d(T_world_grasp)
    pre_grasp_pose_6d = build_pre_grasp_pose_6d(grasp_pose_6d)

    if not moveit_can_reach(pre_grasp_pose_6d):
        continue

    if not moveit_can_reach(grasp_pose_6d):
        continue

    if not moveit_can_plan_to(pre_grasp_pose_6d):
        continue

    return grasp_pose_6d
```

这就是 grasp database 和 MoveIt 最核心的连接方式。

## 5. MoveIt 负责哪些 step

现在 `trajectory_planner.py` 会生成 named steps：

```text
open_gripper_before_approach
move_to_pre_grasp
approach_grasp
close_gripper_at_grasp
hold_after_close
lift_to_pre_grasp
hold_after_lift
transfer_to_drop_high
hold_before_descend
descend_to_drop
open_gripper_to_release
hold_after_release
retreat_from_drop
```

其中 MoveIt 最适合负责这些长距离移动：

```text
move_to_pre_grasp
transfer_to_drop_high
retreat_from_drop
return_to_initial_pose
```

这些动作可能跨越较大空间，应该检查关节限制和碰撞。

短距离动作可以继续用 Cartesian/servo：

```text
approach_grasp
lift_to_pre_grasp
descend_to_drop
```

因为这些动作通常希望沿直线靠近或离开物体。

推荐混合架构：

```text
长距离：MoveIt planning
短距离：Cartesian path / servo
夹爪：GripperCommand action
停顿：hold step
```

## 6. 你现在系统里可能已有的 MoveIt 接口

你之前的 `ros2 action list` 里有：

```text
/arm/move_to_joint
/arm/move_to_pose
/arm/move_to_pose_path
/execute_trajectory
/scaled_joint_trajectory_controller/follow_joint_trajectory
```

这些大概率是 MoveIt 或 MoveIt 封装接口。

可以先看类型：

```bash
ros2 action info /arm/move_to_pose -t
ros2 action info /arm/move_to_pose_path -t
ros2 action info /execute_trajectory -t
```

如果 `/arm/move_to_pose` 是你们 `arm_api2` 封装好的 MoveIt pose action，那么最简单方式是：

```text
executor 执行 move step 时，不再用 servo
而是调用 arm_api2_client.move_to_pose(...)
```

或者使用 action client 给：

```text
/arm/move_to_pose
```

发送目标 pose。

## 7. 当前 servo 执行和 MoveIt 执行的区别

当前 executor 做的是：

```text
move step
 -> interpolate_lin 生成很多小 pose
 -> interpolate_to_pose 一个个发送
```

这是比较直接的末端位姿跟随。

MoveIt 做的是：

```text
目标 pose
 -> IK
 -> collision checking
 -> path planning
 -> JointTrajectory
 -> controller 执行
```

区别：

| 方式 | 优点 | 缺点 |
| --- | --- | --- |
| servo / interpolate_to_pose | 简单，容易调试，直线动作直观 | 碰撞检查弱，可能不可达才失败 |
| MoveIt | 可达性、关节限制、碰撞检查更完整 | 配置和失败原因更复杂 |

## 8. 建议你下一步怎么改代码

不要一下子把所有 move step 都换成 MoveIt。

建议分三步。

第一步：只用 MoveIt 验证候选 grasp。

```text
不执行
只问 pre_grasp / grasp 是否可达
```

第二步：只把 `move_to_pre_grasp` 换成 MoveIt。

```text
当前末端 -> pre_grasp
```

如果这一步稳定，说明 MoveIt 接口和目标 pose 格式是对的。

第三步：把长距离 transfer 换成 MoveIt。

```text
pre_grasp -> drop_high
drop_high -> return/home
```

短距离接近仍然保留 Cartesian。

## 9. 一个更清晰的最终结构

最终可以这样组织：

```text
grasp_selector.py
  读 grasp database
  生成候选 T_world_grasp
  用 MoveIt 检查 IK / planning
  选择 best grasp

trajectory_planner.py
  生成 named steps

executor.py
  move step:
    long move -> MoveIt
    approach/retreat -> Cartesian/servo
  gripper step:
    GripperCommand action
  hold step:
    hold_pose

demo.py
  只负责串起来
```

## 10. 最重要的一句话

5000 个 grasp pose 和 MoveIt 的关系非常大，但它们不是同一个东西：

```text
grasp pose 数据库负责给候选答案
MoveIt 负责判断哪个答案机械臂真的能做到
```

没有 grasp 数据库，MoveIt 不知道“该抓哪里”。

没有 MoveIt，grasp 数据库不知道“机械臂能不能安全到那里”。
