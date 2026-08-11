# 抓取规划教程：从 FoundationPose 到机械臂抓取

这份教程讲的是你的系统里“抓取规划”应该怎么理解、怎么接到现有 pipeline。尽量少写代码，重点放在概念、坐标系、数据流和调试方法。

你的系统目前已经有这些部分：

```text
用户 instruction
 -> gpt-4.1 选择目标物体
 -> SAM2 分割目标物体
 -> FoundationPose 估计目标物体 6D pose
 -> grasps 数据库提供候选抓取姿态
 -> MoveIt / arm_api2 执行抓取
```

其中抓取规划从这一步开始：

```text
FoundationPose pose + grasps 候选姿态
```

目标是算出：

```text
机器人夹爪最终应该移动到哪里、以什么姿态闭合
```

## 1. 抓取规划到底在规划什么

抓取规划不是“识别物体”。识别和定位由前面完成：

- gpt-4.1：决定用户想抓哪个物体
- SAM2：给出物体 mask
- FoundationPose：给出物体 6D pose

抓取规划负责回答：

```text
既然我知道物体在哪里，那夹爪应该从哪个方向接近？
夹爪中心应该放在哪里？
夹爪闭合方向是什么？
有没有碰桌子？
机械臂能不能到？
```

所以抓取规划的输入通常是：

```text
1. 目标物体名称
2. 目标物体当前位姿
3. 该物体的候选抓取姿态
4. 当前机器人状态
5. 桌面和障碍物信息
```

输出通常是：

```text
1. pre-grasp pose
2. final grasp pose
3. gripper width / close command
4. retreat pose
```

## 2. 你已有的 grasps 数据库

你在 `~/Data/grasps` 下看到的目录：

```text
005_tomato_soup_can
009_gelatin_box
013_apple
...
015_peach
...
077_rubiks_cube
```

这是按 YCB 编号组织的预计算抓取数据。

每个 `.npz` 文件里有：

```text
poses: (100, 4, 4) float32
```

这表示每个文件包含 100 个 4x4 抓取位姿矩阵。

这些矩阵可以理解为：

```text
物体坐标系下的夹爪候选姿态
```

也就是：

```text
T_object_gripper
```

注意：这是假设。最终一定要通过可视化或实际测试确认它到底是：

```text
T_object_gripper
```

还是反过来的：

```text
T_gripper_object
```

这一步非常重要。如果方向反了，机器人会去一个完全错误的位置。

## 3. 坐标系是抓取规划的核心

抓取规划最容易出错的地方就是坐标系。

你需要至少理解这些坐标系：

| 坐标系 | 含义 |
| --- | --- |
| `world` / `base_link` | 机器人基座或仿真世界坐标系 |
| `camera` | 深度相机坐标系 |
| `object` | FoundationPose 估计出的物体坐标系 |
| `gripper` / `tcp` | 夹爪或工具中心点坐标系 |

FoundationPose 通常输出：

```text
T_camera_object
```

含义是：

```text
从 object 坐标系变换到 camera 坐标系
```

grasp 数据库通常给：

```text
T_object_gripper
```

含义是：

```text
从 gripper 坐标系变换到 object 坐标系，或者夹爪在 object 坐标系下的姿态
```

你真正要发给机器人的是：

```text
T_base_gripper
```

也就是夹爪在机器人基座坐标系下的目标位姿。

## 4. 最关键的矩阵链

理想情况下，变换链是：

```text
T_base_gripper
  = T_base_camera
  @ T_camera_object
  @ T_object_gripper
```

解释一下：

```text
T_base_camera
```

相机在机器人基座下的位置和姿态。这个应该来自 TF，或者你的 MuJoCo/URDF 静态变换。

```text
T_camera_object
```

FoundationPose 输出的物体位姿。

```text
T_object_gripper
```

grasps 数据库里的候选抓取姿态。

最终得到：

```text
T_base_gripper
```

这是 MoveIt / arm_api2 需要的目标末端位姿。

## 5. 先不要急着真实抓

第一阶段建议只做三件事：

```text
1. 读取 FoundationPose pose
2. 读取一个 grasp pose
3. 计算 T_base_gripper 并打印/保存
```

先不要让机器人动。

原因是你还不知道：

- grasp pose 方向是否正确
- gripper frame 是否等于 robot TCP
- FoundationPose 输出是否在 camera frame
- camera 到 base 的 TF 是否正确
- Z 轴方向是否符合你的夹爪接近方向

如果这些没确认，直接执行抓取很容易撞桌子。

## 6. 如何理解一个抓取姿态矩阵

一个 4x4 pose 矩阵长这样：

```text
[ R11 R12 R13 x ]
[ R21 R22 R23 y ]
[ R31 R32 R33 z ]
[  0   0   0  1 ]
```

右上角：

```text
x, y, z
```

是位置。

左上角 3x3：

```text
R
```

是姿态，也就是夹爪三个轴的方向。

对于抓取来说，你最关心：

```text
夹爪从哪个方向接近物体？
夹爪闭合方向是哪一个轴？
夹爪 palm / tcp 在哪里？
```

不同数据集对 gripper 坐标系定义不同，所以不能只凭感觉。

## 7. pre-grasp 和 final grasp

真实抓取通常不是直接移动到 final grasp。

应该分两步：

```text
pre-grasp pose
 -> final grasp pose
 -> close gripper
 -> lift / retreat
```

### final grasp

final grasp 是真正闭合夹爪的位置。

它来自：

```text
T_base_gripper
```

### pre-grasp

pre-grasp 是沿着夹爪接近方向后退一段距离。

比如后退：

```text
10 cm
```

这样机器人先到物体前方/上方安全位置，再直线靠近。

逻辑上是：

```text
pre-grasp = final grasp 沿 approach 方向反向平移 0.10 m
```

这样比直接冲到 final grasp 安全很多。

## 8. 候选 grasp 不能随便选

每个物体下面有很多 `.npz` 文件，每个 `.npz` 里又有 100 个姿态。候选很多，但不是每个都适合当前场景。

第一版可以先选第一个，用来验证矩阵链。

但真正执行前应该筛选：

```text
1. 接近方向是否合理
2. 是否从桌面上方接近
3. final grasp 是否低于桌面
4. pre-grasp 是否可达
5. final grasp 是否可达
6. 夹爪是否会碰桌子
7. 是否需要绕开其它物体
```

## 9. 最简单的筛选策略

第一版推荐用很简单的规则：

### 规则 1：抓取点不能在桌面以下

如果桌面高度是：

```text
z_table
```

那么 final grasp 的 z 应该满足：

```text
z_gripper > z_table + safety_margin
```

例如：

```text
safety_margin = 0.02 m
```

### 规则 2：接近方向尽量从上往下

桌面抓取最稳定的是从上方接近。

如果某个候选 grasp 会让夹爪从侧面或者桌面下方钻过去，先不要用。

### 规则 3：pre-grasp 要比 final grasp 更远离物体

pre-grasp 应该是安全接近点，不应该更靠近桌子或物体。

### 规则 4：让 MoveIt 试 IK

即使几何上看起来合理，机械臂也不一定能到。

所以最好让 MoveIt 对候选 grasp 做可达性检查：

```text
IK 成功 -> 候选可用
IK 失败 -> 换下一个候选
```

### 规则 5：优先选择离当前末端近的

如果多个候选都可用，选一个移动距离短的，通常更稳定。

## 10. 推荐的抓取执行顺序

一个标准抓取流程可以是：

```text
1. 打开夹爪
2. 移动到 pre-grasp pose
3. 直线移动到 final grasp pose
4. 闭合夹爪
5. 向上抬起 10-15 cm
6. 回到安全位姿
```

对应到你的 pipeline：

```text
FoundationPose 得到 T_camera_object
 -> 读 grasps 得到多个 T_object_gripper
 -> 对每个候选算 T_base_gripper
 -> 筛选可用候选
 -> 执行 pre-grasp
 -> 执行 final grasp
 -> close gripper
 -> lift
```

## 11. 文件名映射

你的 gpt-4.1 / FoundationPose 常用自然语言名字：

```text
tomato soup can
gelatin box
peach
rubiks cube
```

YCB grasp 目录用编号：

```text
005_tomato_soup_can
009_gelatin_box
015_peach
077_rubiks_cube
```

所以你需要一个映射表：

```text
tomato soup can -> 005_tomato_soup_can
gelatin box     -> 009_gelatin_box
peach           -> 015_peach
rubiks cube     -> 077_rubiks_cube
```

这个映射表应该放进 `grasp_selector` 模块里。

## 12. grasp_selector 应该做什么

建议之后写一个：

```text
my_course_pkg/grasp/grasp_selector.py
```

它不需要是 ROS node，普通 Python 类就够。

它负责：

```text
输入：
  object_name
  T_base_object 或 T_camera_object

输出：
  selected T_base_gripper
  selected pre-grasp pose
  selected final grasp pose
```

内部步骤：

```text
1. 根据 object_name 找到 ~/Data/grasps/{ycb_dir}
2. 读取所有 .npz 的 poses
3. 遍历候选 grasp
4. 把 object frame 下的 grasp 转到 base frame
5. 做简单规则筛选
6. 返回第一个可用候选
```

第一版不要追求完美。先做到：

```text
能加载候选
能计算目标 gripper pose
能保存结果
```

再慢慢加筛选规则。

## 13. 如何确认矩阵方向对不对

这是最关键的调试。

建议按下面顺序验证：

### 第一步：只看位置

计算出来的 grasp 位置应该在物体附近。

如果物体在桌面上，gripper 目标位置也应该在桌面上方附近。

如果结果离物体几米远，说明矩阵链错了。

### 第二步：看高度

gripper final pose 的 z 不能明显低于桌面。

如果 z 很低，可能：

```text
grasp pose 方向反了
object pose 坐标系错了
camera->base 变换错了
```

### 第三步：可视化坐标轴

在 RViz 或 MuJoCo 中显示：

```text
object frame
gripper target frame
pre-grasp frame
```

你应该能看到 gripper target 在物体附近，而且朝向合理。

### 第四步：尝试 pre-grasp

只移动到 pre-grasp，不下降、不闭合。

如果 pre-grasp 合理，再尝试 final grasp。

### 第五步：空抓测试

移动到 final grasp，但先不要闭合夹爪，观察是否碰撞。

确认安全后再闭合。

## 14. 常见错误

### 错误 1：把 `T_object_gripper` 当成 `T_gripper_object`

症状：

```text
gripper pose 离物体很远
方向完全不对
```

解决：

```text
尝试对 grasp pose 取 inverse
```

### 错误 2：忘了 camera 到 base 的变换

FoundationPose 很可能在 camera frame 下输出物体 pose。

如果你直接把它当成 base frame，机器人会去错地方。

### 错误 3：gripper frame 和 TCP frame 不一致

grasp 数据里的 gripper frame 不一定等于机器人 URDF 里的 TCP。

可能需要一个固定修正：

```text
T_gripper_tcp
```

最终应该是：

```text
T_base_tcp = T_base_gripper @ T_gripper_tcp
```

### 错误 4：没有 pre-grasp

直接去 final grasp 容易撞物体或桌子。

### 错误 5：只选第一个候选

第一个候选不一定适合当前视角和机械臂姿态。

第一版可以这样做，但最终要筛选。

## 15. 你现在最适合的开发顺序

建议按这个顺序学和写：

```text
1. 读取一个 npz，确认 poses shape 是 (100, 4, 4)
2. 把 poses[0] 打印出来，理解 4x4 矩阵
3. 读取 FoundationPose 输出 pose
4. 手动相乘得到一个 gripper target pose
5. 把 target pose 保存成 json/txt
6. 在 RViz/MuJoCo 里可视化 target pose
7. 增加 pre-grasp
8. 只移动到 pre-grasp
9. 再移动到 final grasp
10. 最后闭合夹爪并抬起
```

不要一开始就做完整自动抓取。先让每一步可解释、可视化、可撤退。

## 16. 最小闭环

最终你的抓取规划最小闭环应该是：

```text
selected_object.json
 -> object_name

FoundationPose
 -> T_camera_object

TF
 -> T_base_camera

grasps
 -> T_object_gripper candidates

grasp_selector
 -> T_base_gripper final
 -> T_base_gripper pre-grasp

arm_api2 / MoveIt
 -> move pre-grasp
 -> move final grasp
 -> close gripper
 -> lift
```

这就是从感知到抓取的完整桥梁。

## 17. 一句话总结

FoundationPose 解决：

```text
物体在哪里
```

`~/Data/grasps` 解决：

```text
这个物体怎么抓
```

抓取规划解决：

```text
从很多候选抓取里，选一个机器人能安全执行的抓取
```

你接下来要写的核心模块就是：

```text
grasp_selector
```

它会把 FoundationPose 的物体位姿和 `grasps` 里的候选姿态组合起来，生成机器人真正要执行的 gripper pose。
