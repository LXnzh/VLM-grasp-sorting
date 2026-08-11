# 精确 OBB 抓取接近通道设计

**日期：** 2026-07-14

**状态：** 已批准并实施；离线测试、构建、无运动 live smoke 和两次正式 Apple A3
plan-only 验证均通过。A3 已完成；Phase B 仍受 `>=18 Hz` 真实运动健康门约束

**当前对象：** Apple（`round_top` 类别代表）

**长期范围：** 五类代表物、同类别其余物体、最终随机位置下的稳定抓取

## 1. 背景与实测问题

Apple Phase A1/A2 已通过，安全的 A3 `grasp_plan_only` 入口也已实现。TF 修复后的
一次非正式 plan-only smoke test 中：

- 成功加载 5 个非目标物体的 `/scene_clearance_bounds`；
- Apple 生成 8 个 round-top 候选；
- 8 个候选全部在发送 MoveIt goal 前被 banana 阻挡；
- 机械臂、夹爪和物体均未运动，`planonly=False` 清理成功。

这不是 FoundationPose、Apple grasp library 或 MoveIt 的失败。当前通道算法把每个
长方体障碍的 XY 投影替换成外接圆：

```text
obstacle_radius_xy = 0.5 * hypot(size_x, size_y)
reject when:
distance(obstacle_center, approach_segment_xy)
    <= obstacle_radius_xy + corridor_radius + clearance_margin
```

banana 的实时 AABB 尺寸约为 `0.1096 x 0.1787 x 0.0373 m`。外接圆半径约
`0.1048 m`，加上 `0.08 m` 通道半径和 `0.03 m` 安全余量后，中心距必须大于
`0.2148 m`。实测 Apple 接近线与 banana 中心的 XY 距离约为 `0.175 m`，因此被拒绝。

但中心到 banana **实际 AABB 长方形边界**的最近距离约为 `0.1207 m`，大于通道与
安全余量之和 `0.1100 m`，约有 `10.7 mm` 的正余量。也就是说，现有外接圆覆盖了
banana 长方形四角之间并不存在物体的区域，造成保守但明显的误拒绝。

这类误拒绝在固定场景可以通过挪动物体暂时绕过，但会直接妨碍最终目标——随机位置下
稳定抓取所有物体。因此本设计不移动 Apple，不减小夹爪通道，也不降低安全余量，而是
修正障碍几何判定。

## 2. 目标

1. 保留 `/scene_clearance_bounds` 发布端的真实、原子、fail-closed AABB 契约。
2. 在 planner 中保留每个 bounds marker 的完整盒体尺寸、中心和方向。
3. 用“接近线段到盒体 XY 长方形的精确最近距离”替代中心到外接圆的距离。
4. 保留现有 `corridor_radius`、`clearance_margin` 和 `vertical_margin`，不调低任何阈值。
5. round-top 场景缺失、TF 无效、盒体无效或精确几何不可用时继续 fail closed。
6. 同一精确通道算法覆盖 `round_top` 与 `side`，不增加 Apple 专用分支。
7. safe placement 继续使用保守外接圆；本轮不改变放置策略。
8. 先解除几何误拒绝，再用两次独立 A3 plan-only 验证 Apple。
9. 为后续多方向接近与随机位置验证建立可复用的几何基础。

## 3. 非目标

- 不移动 Apple、banana 或其他固定槽位。
- 不修改 `ROUND_TOP_APPROACH_CORRIDOR_RADIUS_M=0.08 m`。
- 不修改 `ROUND_TOP_APPROACH_CLEARANCE_MARGIN_M=0.03 m`。
- 不缩短覆盖最低手指包络的 vertical margin。
- 不修改 FoundationPose、SAM2、VLM、grasp library 或 Apple selector 硬门。
- 不修改 MuJoCo `/scene_clearance_bounds` 发布器或重新读取 OBJ。
- 不把 AABB 宣称为 mesh-level 精确碰撞体。
- 不在本轮生成多方向接近候选；它是精确几何后仍有真实阻挡时的下一项设计。
- 不在本轮执行机械臂、Cartesian servo、夹爪、lift、place 或 release。
- 不以单次成功宣告 Apple 稳定；正式 A3 要求连续 2 次独立成功。

## 4. 方案比较

### 4.1 方案 A：精确 2.5D OBB 通道（采用）

把 bounds marker 的盒体完整变换到规划坐标系。将 pregrasp→grasp 线段变换到盒体
局部坐标系，先按 Z 包络裁剪，再计算裁剪后 XY 线段到局部长方形的精确距离。

优点：

- 消除细长物体外接圆造成的空白区域误拒绝；
- 不减小真实物体尺寸，不降低通道半径和安全余量；
- 对竖直接近、侧向接近和带 XY 位移的斜接近使用同一算法；
- 计算量为每个候选、每个障碍的常数时间；
- 只改 planner 与聚焦测试。

边界：它精确对应“已发布盒体 + 水平圆形通道 + 单独垂直包络”的现有 2.5D 安全模型，
不是 mesh collision 或完整 3D 夹爪模型。

### 4.2 方案 B：继续使用外接圆并挪 Apple（不采用）

固定场景中改动最少，但不能解决随机位置下细长障碍普遍误拒绝的问题，也会把场景布局
变成算法正确性的隐式依赖。

### 4.3 方案 C：把盒体再次转换为 world-axis AABB（不采用）

实现简单，但坐标变换后重新取 world AABB 会再次扩大旋转长方形，保留方向信息的收益
会部分丢失。

### 4.4 方案 D：完整 mesh/FCL 或 3D gripper collision（本轮延期）

长期精度最高，但需要夹爪随姿态变化的几何、连续碰撞检测和更多 ROS/MoveIt 集成。
当前问题可由更小、更容易验证的 OBB 通道修复。

## 5. 数据契约

### 5.1 发布端保持不变

`/scene_clearance_bounds` 继续发布：

```text
marker.type             = CUBE
marker.header.frame_id  = base_link
marker.pose.position    = MuJoCo/base_link 轴下 AABB 中心
marker.pose.orientation = identity
marker.scale            = AABB 完整尺寸 [sx, sy, sz]
```

发布端的 AABB 已由 OBJ/geom 顶点和实时姿态计算，且一个周期必须原子包含全部场景物体。
本设计不改变该 topic，也不回读 `/scene_description` 的旧尺寸。

本设计所称“精确盒体/OBB”是指：完整保留已发布 bounds marker 的轴对齐长方体，并将
该长方体通过 TF 刚体变换到 planning world；它不是从原始 mesh 拟合出的最小体积 OBB。
marker frame 与 planning world 一致且 orientation 为 identity 时，它就是精确的 AABB
corridor。

### 5.2 Planner 障碍模型扩展

现有 `SceneClearanceObstacle` 的以下字段保留：

```text
name
center
radius_xy
half_height
```

其中 `radius_xy` 与 `half_height` 继续服务 safe placement 和兼容日志。新增：

```text
T_world_bounds   # 4x4，bounds 局部坐标系到 planning world
half_extents     # [hx, hy, hz]，marker.scale / 2
```

为减少与 safe-placement 单元测试的无关改动，新字段可以有 `None` 默认值；但
approach-corridor 路径不允许使用缺少精确字段的 obstacle。只要进入通道检查而
`T_world_bounds` 或 `half_extents` 缺失，就必须抛出明确错误，不能回退到外接圆。

生产路径由 marker loader 构建 obstacle，因此每个有效 marker 都必须带完整精确几何。

### 5.3 完整 marker pose 组合

不能只变换 marker center。构建：

```text
T_world_bounds = T_world_marker_frame @ T_marker_frame_bounds
half_extents   = 0.5 * [scale.x, scale.y, scale.z]
center         = T_world_bounds[:3, 3]
```

`T_marker_frame_bounds` 必须包含 marker position 和 quaternion orientation。当前发布器给
identity orientation，但 loader 必须正确处理合法的 yaw，避免再次形成只适用于当前数据
的隐式假设。

### 5.4 刚体与轴对齐验证

以下任一条件失败时，整个 obstacle load 失败：

- transform 不是 `4 x 4`；
- position、quaternion、transform 或 scale 包含 NaN/Inf；
- quaternion 范数为零；
- 任一 full extent `<= 0`；
- 齐次矩阵最后一行不是 `[0, 0, 0, 1]`；
- 旋转矩阵不满足正交性或行列式不接近 `+1`；
- bounds 局部 Z 轴与 world Z 轴不平行。

实现复用 `get_transform_checked()` 获取 TF，并复用
`transforms.assert_valid_rotation()` 的 determinant 检查；同时补充 shape、finite、齐次
末行和 `R.T @ R` 正交性检查，因为现有 helper 单独不足以覆盖完整刚体契约。

最后一条是 2.5D 算法的显式前置条件。当前 `base_link -> world` 仅有平移和 yaw，且
bounds marker orientation 为 identity，因此满足该条件。若未来 TF 引入 roll/pitch，
本轮算法不静默近似，而是 fail closed；届时应升级到完整 3D capsule/OBB 距离。

## 6. 精确通道算法

### 6.1 输入

每个候选提供：

- `P0_world`：pregrasp TCP XYZ；
- `P1_world`：grasp TCP XYZ。

每个障碍提供：

- `T_world_bounds`；
- `half_extents=[hx, hy, hz]`。

每个 profile 继续提供：

- `r = corridor_radius_m`；
- `m_xy = clearance_margin_m`；
- `m_z = vertical_margin_m`。

### 6.2 变换到盒体局部坐标

使用受检刚体逆变换：

```text
P0 = inverse(T_world_bounds) * P0_world
P1 = inverse(T_world_bounds) * P1_world
P(t) = P0 + t * (P1 - P0),  t in [0, 1]
```

因为只允许盒体 Z 与 world Z 平行，所以该变换在 XY 平面是旋转或镜像等距变换，水平
圆形通道半径保持不变。

### 6.3 按垂直包络裁剪线段

将障碍局部 Z slab 扩大 vertical margin：

```text
z_min = -hz - m_z
z_max = +hz + m_z
```

计算线段满足 `z_min <= P(t).z <= z_max` 的参数区间
`[t_enter, t_exit]`，并与 `[0,1]` 相交：

- 无交集：该障碍在垂直方向不影响接近线，继续下一个障碍；
- `dz == 0` 且线段 Z 在 slab 外：无交集；
- `dz == 0` 且线段 Z 在 slab 内：使用完整 `[0,1]`；
- 其他情况：按两个 Z 平面求交、排序并裁剪。

这比现有“先比较整条 approach 的总 Z 范围，再对完整 XY 线段求距离”更准确，因为 XY
距离只在真正与障碍垂直包络同时出现的线段部分上计算。

### 6.4 XY 线段到长方形的精确距离

将裁剪后的两个端点投影到局部 XY。障碍长方形为：

```text
R = [-hx, +hx] x [-hy, +hy]
```

计算裁剪 XY 线段到 `R` 的最短欧氏距离 `d_xy`：

1. 若线段与长方形内部或边界相交，`d_xy=0`；
2. 否则计算线段到四条矩形边的 segment-to-segment 距离并取最小值；
3. 退化为点的线段使用 point-to-rectangle 距离；
4. 所有中间值必须有限，否则 fail closed。

拒绝条件：

```text
required = r + m_xy
numeric_epsilon = 1e-9 m
reject if d_xy <= required + numeric_epsilon
```

盒体自身的 `hx/hy` 已由长方形显式包含，不能再把 `radius_xy` 加进 `required`。
`numeric_epsilon` 固定为模块常量 `1e-9 m`，只用于浮点边界的安全侧比较，不读取环境
变量，也不是可调安全余量。

### 6.5 当前 Apple/banana 数值解释

对已捕获的固定场景近似值：

```text
d_xy to banana rectangle    ~= 0.1207 m
corridor radius             =  0.0800 m
clearance margin            =  0.0300 m
remaining geometric margin  ~= 0.0107 m
```

因此该历史样本应通过精确盒体检查，而不是因外接圆被误拒绝。但这只是回归夹具，不提前
保证 fresh pose 下所有候选都能通过。若精确算法对新数据仍拒绝全部候选，就视为真实阻挡，
不得继续降低阈值；下一步进入多方向接近设计。

## 7. 错误与失败关闭策略

### 7.1 场景加载

- `obstacles is None` 的含义继续是 topic 缺失、超时或解析失败；
- `obstacles == []` 继续表示成功加载且排除目标后确实没有其他物体；
- 任一非目标 marker 无法构建精确盒体时，不能返回部分 obstacle list；
- round-top 且 `ROUND_TOP_CLEARANCE_REQUIRE_SCENE=true` 时，`None` 必须阻止所有候选；
- side 现有“topic 整体缺失时警告并跳过”的策略本轮不扩大，但只要场景已加载，任一
  obstacle 精确字段缺失或计算异常都必须报错，不能局部回退到圆。

### 7.2 数值异常

以下情况一律抛出包含 obstacle 名称的错误：

- 逆变换失败；
- 裁剪参数或距离为非有限值；
- half extents 非正；
- 旋转矩阵不满足设计前置条件；
- 通道参数非有限或为负。

不得使用 `0.1 m` cube、零尺寸补偿、外接圆 fallback 或“跳过坏 marker”恢复运行。

## 8. 日志与可诊断性

候选被拒绝时日志至少包含：

- grasp profile 和 candidate index；
- obstacle name；
- `box_xy_distance`；
- `required = corridor_radius + clearance_margin`；
- `clearance = box_xy_distance - required`；
- Z slab 和 `[t_enter, t_exit]`；
- `half_extents`；
- 判定方法标识 `exact_bounds_box_2p5d`，避免把已发布 AABB 经 TF 变换后的盒体误读成
  mesh 拟合 OBB。

汇总继续打印 checked/rejected/remaining。日志不得继续把盒体距离命名为“中心
`xy_distance`”，避免以后误读单位和语义。

## 9. 代码范围

本轮运行代码只修改：

1. `src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
   - 扩展内部 obstacle 数据；
   - 组合并验证完整 marker pose/TF；
   - 新增 Z slab 裁剪和 XY segment-to-rectangle 距离 helper；
   - 替换共享 approach-corridor 判定和日志。
2. `src/my_course_pkg/test/test_pick_place_planner.py`
   - 更新 corridor fixture；
   - 增加精确几何与 fail-closed 回归。

不修改 simulator publisher、YAML 槽位、config 阈值、selector、executor、plan-only 入口、
gripper 或 perception。

## 10. 测试设计

### 10.1 Marker/TF 数据测试

1. identity marker + identity frame：中心、half extents、外接圆字段正确；
2. frame translation + 90° yaw：完整 `T_world_bounds` 正确，half extents 不被交换或重算；
3. 非 identity marker yaw 与 frame yaw 正确组合；
4. target marker 仍被排除；
5. target-only array 仍返回有效空列表；
6. NaN/Inf center、scale、quaternion 或 TF 失败关闭；
7. 零/负 scale 失败关闭；
8. 非正交/非刚体 transform 失败关闭；
9. roll/pitch 使局部 Z 不再平行 world Z 时失败关闭。

### 10.2 几何 helper 测试

1. 点在 rectangle 内，距离为 0；
2. 点在 face 外，距离为到 face 的垂距；
3. 点在 corner 外，距离为到 corner 的欧氏距离；
4. 线段穿过 rectangle，距离为 0；
5. 线段平行于长边且在外侧，距离正确；
6. 退化线段与零 XY 位移行为正确；
7. Z slab 无交集时跳过；
8. vertical margin 刚好产生 Z 交集时进入 XY 检查；
9. 斜线段只在 slab 内的部分参与 XY 距离；
10. 精确边界以及相差不超过固定 `1e-9 m` 的情况按安全侧拒绝。

### 10.3 Planner 行为测试

1. 细长 banana：旧外接圆会拒绝、精确 rectangle 有正余量时保留候选；
2. 当前 Apple/banana 数值夹具得到约 `0.1207 m > 0.1100 m`；
3. 真实靠近 face 的障碍仍拒绝；
4. 真实靠近 corner 的障碍仍拒绝；
5. 90° yaw 后结果随盒体方向正确变化；
6. round-top 缺少场景继续 fail closed；
7. round-top obstacle 缺精确字段失败，不回退到 circle；
8. side 有完整场景时使用同一精确算法；
9. side topic 整体缺失策略保持当前行为；
10. safe placement 现有外接圆选择测试完全不变；
11. planner 集成中全阻挡仍抛出明确错误，至少一个安全候选时正常返回。

### 10.4 回归与构建

在容器中使用 system ROS Python：

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_pick_place_planner.py -q

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_grasp_selector.py \
  src/my_course_pkg/test/test_pick_place_planner.py \
  src/my_course_pkg/test/test_grasp_plan_only.py \
  src/my_course_pkg/test/test_grasp_executor_gripper_failures.py \
  src/my_course_pkg/test/test_trajectory_planner.py \
  src/my_course_pkg/test/test_grasp_eval.py -q

colcon build --packages-select my_course_pkg --symlink-install
```

同时扫描实现，确认没有 Apple 名称分支、槽位改动、阈值改动或 circle fallback。

## 11. 实施顺序（RED → GREEN → 回归）

### Task 1：锁定误拒绝回归

先写失败测试，使用细长 rectangle 和当前 Apple/banana 数值夹具，证明旧外接圆拒绝而
期望精确算法通过。此时不改生产代码。

### Task 2：扩展 obstacle 契约

写 marker pose/TF、half extents、无效刚体和 Z 轴前置条件测试，再实现完整
`T_world_bounds` 构建。保留 safe-placement 字段与行为。

### Task 3：实现纯几何 helper

先覆盖 point/segment-to-rectangle 与 Z slab clipping 的边界测试，再实现无 ROS 依赖的
NumPy helper。helper 不读取环境变量，不认识 object name。

### Task 4：接入共享 corridor

把 `round_top` 和 `side` 的 approach check 切换到精确盒体；删除该路径对
`obstacle.radius_xy` 的依赖；更新 violation 数据和日志；补齐缺精确字段 fail-closed 测试。

### Task 5：离线回归与双工作区同步

运行聚焦测试、相关全回归和 build。只同步上述两个实现/测试文件到
`ros2-docker-workspace-vscode-plmrs-grasp-stable`，使用语义 diff 核对，不覆盖其他 dirty
文件。

### Task 6：只读 live smoke

重启/确认当前 launch 使用新 build，使用已有或 fresh Apple pose 运行一次
`grasp_plan_only` smoke：

- 确认 5 个非目标 bounds 完整加载；
- 确认日志显示 `exact_bounds_box_2p5d`；
- 确认至少一个候选通过几何通道，或记录真实阻挡；
- 确认没有 arm/gripper/object motion；
- 确认 `planonly=False` 清理成功。

该 smoke 不计入正式两次资格。

### Task 7：Apple A3 两次正式验证

每次独立执行：

1. `/reset_sim`；
2. fresh `pipeline`，自然识别 Apple，不使用 candidate/mask override；
3. 确认 FoundationPose 输出属于当前 reset；
4. `GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC=1.5`；
5. 运行 `grasp_plan_only`；
6. 保存 exact corridor、候选 fallback 和 MoveIt plan-only 日志；
7. 比较前后 robot、gripper、Apple pose，确认没有执行运动。

两次都必须满足：fresh bounds 完整、至少一个候选通过精确通道、至少一个 pregrasp
MoveIt plan-only 成功、cleanup 成功、无执行运动。任一次失败都使 A3 连续计数归零。

## 12. 长期随机位置路线

本轮只修复“安全位置被几何近似误拒绝”。后续按以下顺序推进：

1. **Apple 固定位置 A3：** 精确 corridor + MoveIt plan-only，连续 2 次；
2. **Apple 固定位置 B-F：** pregrasp、grasp、close、lift、release 分阶段，每阶段 2 次；
3. **round_top 其余物体：** 复用同一 profile，无物体名分支，逐物体 2 次；
4. **五类代表物：** 各自完成完整稳定闭环；
5. **每类其余物体：** 复用类别 profile，只允许由可解释几何参数解决真实差异；
6. **多方向接近：** 当精确 OBB 判断某方向真实被挡时，生成多个合法接近方向，并用同一
   corridor 与 MoveIt gate 选择；
7. **随机位置：** 先使用带 seed、无初始穿透、在桌面内且可感知的随机分布，再逐步扩大
   位置/朝向范围；每个正式条件仍要求连续 2 次完整成功。

随机位置阶段不得通过关闭 scene gate、降低 margin 或忽略失败 marker 提高表面成功率。
真正不可抓的场景应明确分类为：不可感知、无安全接近方向、IK 不可达或执行失败。

## 13. 完成定义

本轮精确 corridor 实施完成需同时满足：

- `/scene_clearance_bounds` publisher 和槽位没有变化；
- planner 保留完整 bounds pose 和 half extents；
- shared approach path 不再使用 obstacle circumscribed circle；
- safe placement 仍使用保守 circle；
- 当前数值夹具解除外接圆误拒绝；
- face、corner、Z overlap 和 transform 异常测试证明没有引入 fail-open；
- round-top missing/invalid scene 继续 fail closed；
- focused tests、相关回归和 package build 全部通过；
- 两个工作区目标文件语义一致；
- live smoke 无任何执行运动；
- 2 次 fresh Apple A3 plan-only 全部成功；进入 Phase B 前仍必须另行通过
  `>=18 Hz` 真实运动健康门。

如果精确算法正确实现后 fresh Apple 仍被所有障碍真实阻挡，本轮仍视为成功定位真实约束；
下一步必须设计多方向接近，而不是移动固定物体或降低安全阈值。
