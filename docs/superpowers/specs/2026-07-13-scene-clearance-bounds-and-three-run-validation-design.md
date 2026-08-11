# Scene Clearance Bounds 与全阶段三次验证设计

> **政策替代（2026-07-14）：** 本文中所有规范性的连续三次资格门均由
> `2026-07-14-project-wide-two-run-experiment-policy-design.md` 替换为连续两次。
> 本文保留的三次历史实施描述和已完成实验数据仍是事实记录。

**日期：** 2026-07-13

**状态：** 实现与双工作区同步已完成；离线测试、构建、几何正确性和 AABB p95 已通过，但共享发布循环约 `1.48 Hz`，未达到 `>=18 Hz`，Apple A3 继续阻塞

**适用范围：** Apple `round_top` 稳定抓取路线图，以及后续所有类别、物体和随机位置验证

## 1. 背景

Apple Phase A1 已完成三次独立 reset + fresh perception，三次
FoundationPose geometry-center 三维误差分别为 `0.266 mm`、`0.263 mm` 和
`0.245 mm`。Phase A2 已使用 `/home/ws/grasps/013_apple` 运行纯 selector：
5013 个原始 grasp 经 round-top 对称扩展为 40104 个候选，最终正常返回 8 个，
8/8 均通过方向、中心、高度、宽度、开口余量和手指桌面间隙硬门。

进入 A3 plan-only 前发现，现有 `/scene_description` 对所有 YCB mesh 物体都
发布统一 `0.1 x 0.1 x 0.1 m` CUBE。planner 再把它转换为
`radius_xy=0.05 m`、`half_height=0.05 m` 的圆柱障碍。这个尺寸与真实 OBJ
差异明显，例如：

| 物体 | OBJ 包围尺寸（mm） | 当前 marker（mm） |
|---|---:|---:|
| `banana` | `108.9 x 178.4 x 36.7` | `100 x 100 x 100` |
| `hammer` | `182.2 x 332.7 x 32.9` | `100 x 100 x 100` |
| `apple` | `75.4 x 74.9 x 71.9` | `100 x 100 x 100` |
| `tomato_soup_can` | `67.9 x 67.7 x 101.9` | `100 x 100 x 100` |

此外，YCB body origin 通常不在几何中心。以 hammer 为例，其 OBJ AABB 中心
相对 body origin 约偏移 `[-37.7, -22.7, 15.8] mm`。因此只替换 scale 而保留
body-origin marker center 仍然不正确。

`/scene_description` 现有 pose 同时被 A1 和 `SimClientNode` 当作 YCB body
origin/姿态真值，不能直接改为 AABB center。设计采用两个 topic 分离 pose
语义和占用空间语义。

## 2. 目标

1. 保持 `/scene_description` 的 body-origin/姿态契约完全不变。
2. 新增 `/scene_clearance_bounds`，发布每个场景物体基于真实 MuJoCo geometry
   和实时姿态的世界轴 AABB。
3. round-top corridor、side corridor 和 safe placement 统一使用新 bounds。
4. 禁止用固定 `0.1 m`、零尺寸自动拔高或其他猜测替代无效 AABB。
5. round-top 在 bounds 缺失或无效时继续 fail closed。
6. 将整个稳定抓取项目的资格判定从连续 5 次统一改为连续 3 次；一次完整
   循环内部的 5 秒 lift hold 保持不变。

## 3. 非目标

- 本改动不执行 A3、MoveIt 轨迹、机械臂运动或夹爪命令。
- 不在 planner 中维护 YCB 物体专用尺寸表。
- 不修改 FoundationPose、selector 硬门或 Apple 抓取 offset。
- 不把 AABB 当作精确碰撞网格；AABB 和其 XY 外接圆有意保持保守。
- 不在本轮改变 side profile 现有的“场景缺失时跳过 corridor”政策，只迁移其
  数据源并修复有数据时的尺寸正确性。

## 4. Topic 契约

### 4.1 `/scene_description`

保持现状：

- 类型：`visualization_msgs/msg/MarkerArray`；
- marker pose：YCB body origin 和实时 body orientation；
- frame：现有 `base_link`；
- 消费者：A1 truth、`SimClientNode` 和其他现有 object-pose 消费者。

本改动不得改变 marker 数量、object text、pose、frame 或现有消费者行为。

### 4.2 `/scene_clearance_bounds`

新增 `visualization_msgs/msg/MarkerArray` publisher。每个已选场景物体恰好一个
marker：

```text
marker.ns               = "scene_clearance_bounds"
marker.text             = canonical object name
marker.type             = Marker.CUBE
marker.header.frame_id  = "base_link"
marker.pose.position    = AABB center in simulator/base_link axes
marker.pose.orientation = identity
marker.scale            = [size_x, size_y, size_z] full extents
```

每个 marker 使用同一发布周期时间戳。scale 三轴必须有限且严格大于零。topic
使用与 `/scene_description` 相容的可靠、volatile QoS，以便 planner 获取当前
帧而不是持久化旧场景。

MuJoCo 仿真坐标由现有 TF 映射到 ROS `world`。当前映射仅含平移和绕 Z 的
旋转，planner 使用的 XY 外接圆半径对该旋转不敏感，Z half-height 也保持
不变。publisher 不伪装自己拥有 ROS TF；它在 `base_link` 中发布 simulator
world-axis AABB，planner 沿用现有受检 TF 转换中心。

## 5. MuJoCo 侧 AABB 数据流

### 5.1 几何缓存

仿真模型加载时，为每个已选 object body 建立 body-local geometry cache：

- 复用/抽取现有 `_compute_mesh_placement_info()` 的 OBJ vertex loader，避免
  placement 与 bounds 分别实现两套 mesh 解析；
- 枚举 YCB XML 中属于 object body 的 geoms，并解析其 mesh asset；
- 在初始化阶段对原始 OBJ vertex 应用 `<mesh scale>` 和 geom-local
  `pos/quat/euler`，把结果一次性烘焙成 body-local vertices；当前 YCB assets
  使用 identity geom transform 和 `scale="1 1 1"`，reference test 必须覆盖
  非单位 scale 与至少一种 geom-local rotation；
- 同一物体的 visual 与 collision mesh 都纳入 union，使结果至少覆盖两者；
- 顶点数组在初始化阶段缓存，发布周期不得重新读取 XML 或 OBJ；
- 遇到未受测试支持的 mesh transform attribute 时启动失败，禁止静默忽略或
  猜测 MuJoCo 编译后 mesh frame 的补偿关系。

box、sphere 和 cylinder 继续受支持：box 使用旋转矩阵绝对值投影 half-extents，
sphere 使用半径，cylinder 使用其轴向 half-height 与径向半径计算各世界轴
extent。一个 object 拥有多个 geom 时，对全部 geom 的 min/max 取 union。

初始化时遇到不存在的 body、无 geom、空 mesh、非有限 vertex 或无法建立
正尺寸 local geometry，启动必须以包含 object/geom/asset 名称的明确异常失败。

### 5.2 每周期实时计算

每次 scene publish：

1. 读取 object body 的实时 `data.xpos` 与 `data.xmat`；
2. 用 NumPy 向量化转换已烘焙的 body-local vertices；
3. 对全部 world vertices/extents 取 min/max；
4. `center=(min+max)/2`，`size=max-min`；
5. 验证 center/size 有限且三轴 `size > epsilon`；
6. 生成 identity-orientation CUBE marker。

本轮选择真实 vertex transform，不先用“旋转 local AABB 八角点”优化。后者
虽然仍保守，但更松；只有测量证明实时 vertex transform 影响控制/发布节拍时，
才另行设计优化。

### 5.3 原子发布与运行时错误

一个周期内必须成功计算全部已选物体的 bounds 才发布 MarkerArray。不得发布
缺少某个物体的部分列表，因为 planner 无法从单独的 bounds topic 判断漏项。

运行时若出现非有限 pose、无效尺寸或 cache/geom 不一致：

- 本周期整个 bounds array 不发布；
- 输出包含 object 和失败原因的节流 error；
- 不发布固定 `0.1 m` fallback，也不静默跳过单个物体。

下一个健康周期可以恢复发布。round-top consumer 在超时后自然 fail closed。

## 6. Planner 迁移

现有共享 scene-clearance loader 改为订阅 `/scene_clearance_bounds`。round-top、
side 和 safe placement 不再从 `/scene_description` 读取 scale。

对每个非目标 marker：

```text
radius_xy  = 0.5 * hypot(scale.x, scale.y)
half_height = 0.5 * scale.z
```

`hypot` 外接圆覆盖 AABB 四角，适用于 corridor 与现有圆形 safe-placement
clearance 近似。删除 scale 非正时自动替换为 `0.05 m` 的 fallback；无效 marker
使本次 obstacle load 失败。

目标排除继续使用规范化 `marker.text`。成功加载后，如果场景确实只有目标物体，
排除目标得到空 obstacle list 仍是有效状态，必须与 topic 缺失/解析失败的
`None` 区分。

失败政策：

- `round_top` 且 `ROUND_TOP_CLEARANCE_REQUIRE_SCENE=true`：topic 缺失、无效
  或解析失败全部 fail closed；
- `side`：迁移到新 topic，有数据时使用真实 bounds；缺失时保留现有显式警告
  后跳过 corridor 的行为；
- safe placement：继续服从 `GRASP_PLACE_REQUIRE_SCENE`，不得回读旧 topic
  作为尺寸 fallback。

## 7. 全阶段连续三次规则

所有稳定性资格门统一为一个新定义：

> 从 reset 开始，连续完成 3 次完整的 fresh perception -> grasp -> lift ->
> 5 秒 hold -> safe place -> verified release -> retreat/return 循环，且无人工
> 修正。任何失败都使当前连续计数归零。

应用范围：

1. 每个抓取类别的代表物；
2. 同类别的每个其他物体；
3. 取消固定位置后的每个要求位置/随机化条件。

实施时把 Apple roadmap 中“连续 5 次”改为“连续 3 次”，把
`grasp_eval.py --trials` 默认值从当前 `10` 改为 `3`。一个默认资格批次必须
3/3 成功；若失败，后续资格批次从 0 重新开始。显式传入 `--trials` 仍允许
更长的压力测试，但不能降低正式资格门的 3 次要求。

`GRASP_DEBUG_STOP_AFTER_LIFT` 所使用的 5 秒主动保持，以及完整执行中的同等
hold，不属于“次数”参数，本设计不缩短。

## 8. 测试设计

### 8.1 MuJoCo 离线单元测试

- synthetic off-origin mesh：验证 AABB center 不等于 body origin 时仍正确；
- identity、90 度 yaw、任意 roll/pitch：输出与直接 brute-force world vertex
  min/max 一致；
- 多 mesh geom union：visual 与 collision 中任一外扩都被包含；
- box、sphere、cylinder world-axis extent；
- 空 vertex、NaN/Inf、零尺寸和缺失 body/geom 明确失败；
- 运行时单个物体失败时不发布部分 array。

### 8.2 Topic 契约测试

- `/scene_description` 的 marker pose/text/frame 保持不变；
- `/scene_clearance_bounds` 每个 object 恰好一个 CUBE；
- marker orientation 为 identity、scale 为 full extents、center 为 world AABB
  center；
- 当前 YCB reference assets 的 identity bounds 与 OBJ/MuJoCo reference 匹配；
- banana 和 hammer 明确不再是 `0.1 m` cube。

### 8.3 Planner 测试

- `radius_xy=0.5*hypot(sx,sy)`，而不是 `0.5*max(...)`；
- round-top、side 与 safe placement 均订阅新 topic；
- target exclusion、loaded-empty 与 missing/invalid 的三种状态区分；
- round-top missing/invalid fail closed；
- side 现有 missing-scene 行为保持；
- 现有 round-top near/far、side near/far corridor 行为在使用真实 bounds 后
  仍满足各自安全断言。

### 8.4 回归与 live 只读验证

- 运行 simulator scene-selection/AABB tests；
- 运行现有 focused grasp selector/planner/executor/trajectory tests；
- `colcon build --packages-select my_course_pkg --symlink-install`；
- relaunch simulator 后同时读取两个 topic：确认 `/scene_description` pose 不变，
  `/scene_clearance_bounds` 的 hammer/banana center 和 scale 符合实时姿态；
- 记录 AABB 计算耗时和 scene publish 节拍；若出现可测量退化，停止进入 A3，
  不在本轮临时换成不受审查的近似；
- 上述验证不运行 MoveIt、不执行轨迹、不发送夹爪命令。

## 9. 跨仓库同步

实施以 `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable` 活动工作区为先，
验证通过后把相关 simulator、planner、tests 和文档语义同步到
`E:\IFL\ros2-docker-workspace-vscode-plmrs`。同步后使用 semantic diff，忽略
CRLF 差异，不覆盖任一仓库的无关 dirty changes。

## 10. Definition of Done

只有全部满足时，本设计实现才完成：

1. `/scene_description` body-origin/姿态契约无回归；
2. `/scene_clearance_bounds` 原子发布全部场景物体实时 AABB；
3. hammer、banana 等 mesh 的 center/scale 来自真实 geometry，不再使用
   `0.1 m` fallback；
4. planner 三个消费者全部迁移，XY 半径覆盖 AABB 角点；
5. round-top bounds 缺失/无效继续 fail closed；
6. 所有新增测试、focused regression 和 build 通过；
7. live topic-only 检查通过且发布节拍无不可接受退化；
8. 全阶段文档与默认 evaluator 使用连续 3 次资格门；
9. 两个仓库完成受控同步；
10. 在以上条件满足前，不运行 Apple A3 plan-only，更不进入 Phase B 实际运动。
