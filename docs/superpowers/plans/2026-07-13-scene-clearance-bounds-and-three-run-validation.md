# Scene Clearance Bounds 与全阶段三次验证实施计划

> **政策替代（2026-07-14）：** 本计划中所有规范性的连续三次资格门均由
> `../specs/2026-07-14-project-wide-two-run-experiment-policy-design.md` 替换为连续两次。
> 旧任务步骤保留为当时的实施记录，不再定义当前资格门。

> **给实施者：** 按任务顺序执行，每个任务都遵循 RED -> GREEN -> 回归验证。任何非预期测试失败、live topic 缺项或性能门失败都停止，不得绕过 `/scene_clearance_bounds`，也不得提前进入 Apple A3、MoveIt 或机械臂/夹爪运动。

**目标：** 在保持 `/scene_description` body-origin/姿态语义不变的前提下，新增基于真实 OBJ/geom 和实时 MuJoCo 姿态的 `/scene_clearance_bounds`，让 round-top corridor、side corridor 和 safe placement 使用保守世界轴 AABB；同时把项目资格门统一为连续 3 次完整成功。

**架构：** 仿真端在初始化时把每个已选物体的 mesh/primitive 几何缓存到 body-local 坐标；发布周期只读取实时 `data.xpos`/`data.xmat` 并计算世界轴 AABB。ROS 层以独立 `MarkerArray` 原子发布 bounds，规划器共享 loader 只订阅该 topic，并把 AABB 转为覆盖四角的 XY 外接圆。旧 `/scene_description` 继续只承担物体 body pose 真值。

**技术栈：** Python 3.10、NumPy、MuJoCo、rclpy、`visualization_msgs/MarkerArray`、pytest、ROS 2 Humble、colcon。

**实施主工作区：** `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`，容器内为 `/home/ws`。

**受控同步工作区：** `E:\IFL\ros2-docker-workspace-vscode-plmrs`。先在实施主工作区完成全部离线和 live topic-only 验证，再逐文件语义同步，禁止覆盖两边无关 dirty changes。

**状态：** Task 0-5 与受控同步已完成；65 项模拟器测试、92 项抓取/评估测试和 `my_course_pkg` 构建通过。live AABB p95 为 `4.756 ms`，但同循环 topic 约 `1.48 Hz`，未达到 `>=18 Hz`，因此性能门未全部通过且 Apple A3 继续阻塞。

---

## 1. 当前基线与硬门

- Apple A1 已通过 3 次独立 reset + fresh perception。
- Apple A2 已通过纯 selector，8/8 最终候选满足硬门。
- 当前 `/scene_description` 对 mesh 仍发布固定 `0.1 x 0.1 x 0.1 m`，不能用于可信 corridor/safe-place clearance。
- 当前目标文件在两个工作区语义一致；两边都存在无关 dirty/untracked 文件，实施者必须保留。
- 本计划完成前只允许离线单元测试、build、仿真重启和 topic-only 读取。
- 禁止运行：`grasp_demo`、Apple A3 plan-only、MoveIt 轨迹、机械臂命令、夹爪命令。

### Definition of Done

只有同时满足以下条件才可解除 A3 前置阻塞：

1. `/scene_description` 的 marker 数量、text、frame、body-origin position 和 body orientation 无回归。
2. `/scene_clearance_bounds` 每周期包含全部已选物体且每个物体恰好一个 world-axis CUBE。
3. mesh center/scale 来自真实几何；hammer/banana 不再是固定 `0.1 m` cube。
4. 任一物体 bounds 失败时，本周期不发布整个 bounds array，不发布部分列表。
5. round-top、side、safe placement 都只从新 topic 读取 clearance 尺寸。
6. `radius_xy = 0.5 * hypot(size_x, size_y)`；无效尺寸不再自动替换为 `0.05 m`。
7. round-top 缺失/无效 bounds fail closed；side 和 safe placement 保持已批准的各自缺失场景策略。
8. 新增测试、focused grasp 回归和 `my_course_pkg` build 全部通过。
9. live topic-only 验证达到 `>=18 Hz`，AABB 计算 p95 `<=10 ms`，且无持续 loop stall。
10. evaluator 和路线图使用连续 3 次门；5 秒 lift hold 保持不变。
11. 两个工作区完成受控语义同步，handoff 更新。

---

## 2. 文件结构

### 新增

- `src/ifl_air_mujoco_sim/env/utils/scene_clearance_bounds.py`
  负责 OBJ 读取、YCB XML/geom 解析、body-local 几何缓存和 world-axis AABB 数学。
- `src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py`
  覆盖 synthetic mesh、transform、union、primitive、错误和 YCB reference assets。
- `src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py`
  覆盖两个 topic 的 marker 契约、单时间戳和原子不发布行为。

### 修改

- `src/ifl_air_mujoco_sim/env/utils/populate_scene.py`
  复用统一 OBJ vertex loader，避免 placement 与 bounds 各写一套读取器。
- `src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`
  初始化 geometry/body-id cache，提供原子的 `get_scene_clearance_bounds()`。
- `src/ifl_air_mujoco_sim/env/ros2_interface.py`
  新建 publisher、构造 CUBE MarkerArray、节流错误并在现有 scene cadence 发布。
- `src/my_course_pkg/my_course_pkg/grasp/config.py`
  新增专用 topic 常量，保留现有 timeout 行为。
- `src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
  迁移 shared obstacle loader，严格解析 scale，采用 AABB 外接圆半径。
- `src/my_course_pkg/test/test_pick_place_planner.py`
  更新 topic 断言、尺寸算法、missing/invalid/loaded-empty 和三消费者回归。
- `src/my_course_pkg/my_course_pkg/grasp_eval.py`
  默认 3 次并报告连续成功资格状态。
- `src/my_course_pkg/test/test_grasp_eval.py`
  验证默认次数、显式长测和失败归零的连续计数。
- `docs/superpowers/specs/2026-07-13-apple-stable-grasp-roadmap-design.md`
  将资格门从 5 次统一为 3 次，不修改“五类”和“5 秒 hold”。
- `docs/superpowers/specs/2026-07-13-scene-clearance-bounds-and-three-run-validation-design.md`
  状态改为已批准/进入实施。
- `docs/agent_handoff.md`
  两个工作区分别记录实施、验证结果和下一步。

---

## Task 0：保护 dirty worktree 并记录 RED 前基线

**文件：** 无实现修改。

- [ ] **Step 1：记录两个工作区状态**

```powershell
git -C E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable status --short
git -C E:\IFL\ros2-docker-workspace-vscode-plmrs status --short
```

把输出保留在会话记录中。不得 reset、checkout、clean 或覆盖无关文件。

- [ ] **Step 2：确认容器和依赖组合**

在容器中，系统 Python 有 pytest/ROS，MuJoCo 在 simulator venv。测试统一使用：

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim:/home/ws/src/ifl_air_mujoco_sim/.venv/lib/python3.10/site-packages:/home/ws/src/my_course_pkg:$PYTHONPATH
```

- [ ] **Step 3：跑改动前 focused 基线**

```bash
python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/my_course_pkg/test/test_grasp_selector.py \
  src/my_course_pkg/test/test_pick_place_planner.py \
  src/my_course_pkg/test/test_grasp_executor_gripper_failures.py \
  src/my_course_pkg/test/test_trajectory_planner.py \
  src/my_course_pkg/test/test_grasp_eval.py -q
```

预期：全部通过。若基线失败，先定位是否为已有环境/dirty change，不把它混入本实现。

---

## Task 1：建立统一 OBJ/XML 几何缓存模块

**文件：**

- 新增：`src/ifl_air_mujoco_sim/env/utils/scene_clearance_bounds.py`
- 新增测试：`src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py`
- 修改：`src/ifl_air_mujoco_sim/env/utils/populate_scene.py`

### 1.1 数据结构和公开接口

新模块定义不可变数据结构，名称在实现中固定：

- `MeshGeometry(vertices_body)`：已经应用 mesh scale 和 geom-local transform 的 body-local 顶点。
- `BoxGeometry(center_body, rotation_body, half_extents)`。
- `SphereGeometry(center_body, radius)`。
- `CylinderGeometry(center_body, rotation_body, radius, half_height)`。
- `ObjectGeometryCache(object_name, geometries)`。
- `SceneClearanceBound(name, center, size)`。

公开函数：

- `load_obj_vertices(obj_path)`。
- `build_object_geometry_cache(obj_config)`。
- `compute_world_aabb(cache, body_position, body_rotation, epsilon=1e-9)`。

模块不导入 rclpy，也不依赖运行中的 MuJoCo model；body ID/pose 由 `MuJoCoInterface` 提供。

### 1.2 先写失败测试

- [ ] **Step 1：OBJ loader RED**

新增：

- `test_load_obj_vertices_reads_only_vertex_records`
- `test_load_obj_vertices_rejects_empty_and_nonfinite_mesh`

断言：支持空白/注释/face 行；空顶点、NaN/Inf、少于三列或缺失文件都抛出包含路径的 `ValueError`/`FileNotFoundError`，不得返回零尺寸占位。

- [ ] **Step 2：synthetic YCB XML RED**

使用 `tmp_path` 创建两个小 OBJ 和一个 XML，覆盖：

- visual + collision 两个 mesh geom 都进入 union；
- `<mesh scale="2 0.5 1.5">`；
- geom-local `pos`；
- 一个 `quat` rotation fixture；
- 一个 `euler` rotation fixture。

测试名：

- `test_mesh_cache_applies_asset_scale_and_geom_translation`
- `test_mesh_cache_applies_mujoco_quaternion_rotation`
- `test_mesh_cache_applies_default_degree_euler_rotation`
- `test_mesh_cache_unions_visual_and_collision_geometries`

MuJoCo XML 默认 `compiler angle="degree"`；若 XML 写 `angle="radian"` 则按弧度。当前实现只接受默认 `eulerseq="xyz"`，其他 sequence 明确失败，避免猜测编译器语义。

- [ ] **Step 3：unsupported/invalid RED**

新增参数化测试：

- 精确 object body 不存在；
- body 无 geom；
- geom 引用不存在的 mesh asset；
- 空 mesh；
- 非正/非有限 scale；
- geom 同时声明多个 orientation 属性；
- 零范数 quat；
- 当前未支持的 `axisangle`、`xyaxes`、`zaxis`、`fromto`、mesh `refpos/refquat`；
- 未处理的 nested child body。

统一断言异常包含 object、geom 或 asset 名，启动时 fail fast。

- [ ] **Step 4：运行 RED**

```bash
python3 -m pytest src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py -v
```

预期：因新模块/接口不存在而失败。

### 1.3 实现 geometry loader

- [ ] **Step 5：实现 `load_obj_vertices()`**

逐行解析 `v x y z`，返回 `float64 (N,3)`；读取后一次性检查维度、非空和 finite。禁止在发布周期调用该函数。

- [ ] **Step 6：实现 XML transform 解析**

规则固定为：

1. 必须找到 `worldbody/body[@name=obj_config['name']]`。
2. 建立 `<asset><mesh name=...>` 映射；mesh file 相对 XML 目录解析。
3. 原始 OBJ 顶点先逐轴乘 asset scale。
4. 再应用 geom-local rotation 和 `pos`：`v_body = R_geom @ v_scaled + p_geom`。
5. MuJoCo quat 顺序为 `[w,x,y,z]`，规范化后转矩阵。
6. euler 遵从 XML compiler angle，默认 degree；只接受已测试的 XYZ sequence。
7. body 直属全部 mesh geoms（visual 与 collision）都拼接进一个 union vertex array。
8. 当前不静默遍历 nested body；检测到后明确报 unsupported，避免漏掉父子 transform。

- [ ] **Step 7：实现 primitive cache**

对 config 中 `box`、`sphere`、`cylinder` 直接构造 analytic descriptor，沿用现有 `size` 语义：

- box `size=[hx,hy,hz]`；
- sphere `size=[radius]`；
- cylinder `size=[radius,half_height]`。

所有值必须 finite 且严格大于零。

### 1.4 实现 world-axis AABB 数学

- [ ] **Step 8：先写 transform/primitive RED**

新增：

- `test_mesh_world_aabb_matches_bruteforce_at_identity`
- `test_mesh_world_aabb_matches_bruteforce_at_ninety_degree_yaw`
- `test_mesh_world_aabb_matches_bruteforce_at_arbitrary_roll_pitch_yaw`
- `test_off_origin_mesh_reports_geometry_center_not_body_origin`
- `test_box_world_extent_uses_absolute_rotation_projection`
- `test_sphere_world_extent_is_rotation_invariant`
- `test_cylinder_world_extent_uses_axis_and_radial_projection`
- `test_world_aabb_rejects_nonfinite_pose_and_nonpositive_extent`

- [ ] **Step 9：实现公式**

Mesh 使用真实顶点：

```text
V_world = (R_body @ V_body.T).T + p_body
minimum = min(V_world, axis=0)
maximum = max(V_world, axis=0)
center  = 0.5 * (minimum + maximum)
size    = maximum - minimum
```

Primitive 使用 analytic extent：

```text
box:      half_world = abs(R_world_geom) @ half_extents
sphere:   half_world = [r, r, r]
cylinder: axis = R_world_geom[:, 2]
          half_world_i = h*abs(axis_i) + r*sqrt(max(0, 1-axis_i^2))
```

多个 geometry 的 min/max 再 union。最终 center/size 必须 finite，三轴 `size > epsilon`。

### 1.5 复用 loader 并验证 reference assets

- [ ] **Step 10：重构 placement OBJ 读取**

在 `populate_scene.py::_compute_mesh_placement_info()` 中删除内联 `v ` 解析，改为调用 `load_obj_vertices()` 取得 **raw vertices**，然后仍由 `_compute_mesh_placement_info()` 独立应用传入的 `orientation`，再计算 rotated centroid 和 `z_min`。保持当前已验证 YCB placement 的 centroid/orientation 语义；当前资源 scale 全为 `1 1 1`，不得借本任务改变固定槽位或 body pose。增加回归断言，证明相同 XML/orientation 的 placement 结果在重构前后数值不变。

- [ ] **Step 11：加入真实 YCB reference tests**

从相邻 `src/my_course_pkg/YCB_Dataset/ycb` 定位 apple、banana、hammer XML，不硬编码 `/home/ws`。断言：

- 三者 cache 成功且包含 visual/collision union；
- identity AABB 与独立 brute-force reference 在容差内一致；
- Apple 尺寸约为 `75.4 x 74.9 x 71.9 mm`；
- banana/hammer 至少明确不等于 `[0.1,0.1,0.1] m`；
- hammer off-origin center 不等于 body origin。

- [ ] **Step 12：运行 GREEN 和 placement 回归**

```bash
python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py -v
```

预期：全部通过。

---

## Task 2：把 geometry cache 接入 `MuJoCoInterface`

**文件：**

- 修改：`src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`
- 测试：`src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py`

- [ ] **Step 1：写接口 RED**

使用 `MuJoCoInterface.__new__()`、fake `data.xpos/xmat` 和预构建 cache，新增：

- `test_mujoco_interface_returns_all_bounds_in_selected_order`
- `test_mujoco_interface_uses_live_body_pose_each_call`
- `test_mujoco_interface_raises_if_any_body_or_cache_is_missing`
- `test_mujoco_interface_discards_local_partial_result_on_runtime_error`

最后一条断言 API 抛错且调用方拿不到任何 list，而不是返回前几个物体。

- [ ] **Step 2：初始化 cache 和 body IDs**

在 `MjModel`/`MjData` 创建后：

1. 按 `self.objects_config` 顺序调用 `build_object_geometry_cache()`。
2. 用 `mujoco.mj_name2id(..., mjOBJ_BODY, obj_name)` 解析 body ID。
3. body ID 为 `-1`、重复 object name、cache 构建失败时直接中止启动。
4. 缓存 XML/OBJ 解析结果和 body ID；不得在 publish loop 读磁盘。

- [ ] **Step 3：实现 `get_scene_clearance_bounds()`**

每次调用：

1. 按已选物体顺序读取对应 `data.xpos[body_id]` 和 `data.xmat[body_id].reshape(3,3)`。
2. 调用 `compute_world_aabb()`。
3. 先在局部 list 完成全部物体；全部成功后才 return。
4. 任一失败抛出包含 object name 的 `RuntimeError`，不 `continue`、不补 `0.1 m`。

现有 `get_scene_description()` 不重构，不改变其容错和 body pose 输出。

- [ ] **Step 4：运行 GREEN**

```bash
python3 -m pytest src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py -v
```

---

## Task 3：新增原子 `/scene_clearance_bounds` publisher

**文件：**

- 修改：`src/ifl_air_mujoco_sim/env/ros2_interface.py`
- 新增测试：`src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py`

- [ ] **Step 1：写 marker 契约 RED**

新增：

- `test_scene_description_pose_text_and_frame_contract_is_unchanged`
- `test_clearance_bounds_builds_one_identity_cube_per_object`
- `test_clearance_bounds_uses_one_stamp_and_full_extents`

新 marker 断言：

```text
ns = scene_clearance_bounds
text = canonical object name
type = Marker.CUBE
frame_id = base_link
pose.position = AABB center
pose.orientation = [x=0,y=0,z=0,w=1]
scale = full [size_x,size_y,size_z]
action = ADD
lifetime = 0
```

同一 array 所有 marker 使用同一个预先取得的 stamp；ID 按 selection order 稳定。

- [ ] **Step 2：写原子发布 RED**

新增：

- `test_clearance_publisher_publishes_complete_array_only_after_success`
- `test_clearance_publisher_does_not_publish_when_one_bound_fails`
- `test_clearance_publisher_throttles_repeated_runtime_errors`

用 fake sim/publisher/logger 构造 `UR10eRos2Interface.__new__()`，断言失败时 publisher 调用次数保持 0。

- [ ] **Step 3：创建 publisher**

在现有 reliable/volatile/depth-1 QoS 上新增：

```text
/scene_clearance_bounds : visualization_msgs/msg/MarkerArray
```

保留 `/scene_description` publisher 和方法不变。

- [ ] **Step 4：实现构造与发布方法**

增加独立方法：

- `_create_scene_clearance_bounds_marker_array(bounds, stamp)`：只做完整 list -> message 转换。
- `_publish_scene_clearance_bounds()`：调用 sim、构造、publish；成功返回 `True`，失败返回 `False`。

在现有 joint-state/scene publish cadence 中，以独立 `try` 调用 bounds publisher。旧 scene 发布失败不应伪装成 bounds 成功，bounds 失败也不应改变旧 topic。

- [ ] **Step 5：节流 runtime error**

失败日志必须包含 object/原因和“本周期未发布完整 array”。同一错误最多每 2 秒输出一次；下个健康周期自动恢复。禁止在 exception handler 中创建空 array、部分 array 或固定 cube。

- [ ] **Step 6：加入一次性性能采样**

只对前 100 次成功 `get_scene_clearance_bounds()` 计时，随后一次性记录 count/median/p95/max 并停止采样，避免永久日志噪声。该数据用于 live DoD，不改变控制频率。

- [ ] **Step 7：运行 GREEN**

```bash
python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py -v
```

---

## Task 4：迁移 planner 的 shared obstacle loader

**文件：**

- 修改：`src/my_course_pkg/my_course_pkg/grasp/config.py`
- 修改：`src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
- 修改测试：`src/my_course_pkg/test/test_pick_place_planner.py`

- [ ] **Step 1：写 topic/算法 RED**

新增或更新：

- `test_scene_clearance_loader_subscribes_only_to_bounds_topic`
- `test_scene_clearance_radius_covers_aabb_corners`
- `test_scene_clearance_half_height_uses_full_z_extent`
- `test_scene_clearance_rejects_zero_negative_nan_or_inf_scale`
- `test_scene_clearance_rechecks_center_after_tf_transform`
- `test_scene_clearance_target_only_array_returns_loaded_empty_list`

使用 `scale=(0.06,0.08,0.10)` 时预期：

```text
radius_xy = 0.5 * hypot(0.06, 0.08) = 0.05 m
half_height = 0.05 m
```

这能区分旧 `0.5*max(...) = 0.04 m`。

- [ ] **Step 2：新增 topic 常量**

在 `config.py` 新增：

```text
GRASP_SCENE_CLEARANCE_TOPIC
default = /scene_clearance_bounds
```

保留当前 `GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC` 数值和环境变量作为共享 scene wait timeout，避免在本轮同时制造 timeout 配置迁移；代码注释说明它现在用于 clearance bounds 等待。不得回读旧 topic。

- [ ] **Step 3：泛化 wait helper**

把硬编码 `/scene_description` 的 `_wait_for_scene_marker_array()` 改为显式接收 topic 的 `_wait_for_marker_array(node, topic, timeout_sec)`，或等价的专用 bounds helper。`_load_scene_clearance_obstacles()` 只能传 `GRASP_SCENE_CLEARANCE_TOPIC`。

所有 warning/RuntimeError 文本改为 `/scene_clearance_bounds`，便于现场诊断。

实施后执行残留扫描：

```bash
grep -n 'scene_description' src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py
```

预期：shared clearance loader、订阅调用和错误文本中无旧 topic；若仍有命中，必须逐条确认它是否属于真正的 pose consumer，而不是 clearance fallback。

- [ ] **Step 4：严格转换 marker**

在 `_marker_to_clearance_obstacle()`：

1. position 三轴先检查 finite。
2. TF 转换后再次检查 world center finite。
3. scale 不取 `abs()`；原值必须 finite 且三轴严格 `>0`。
4. `radius_xy=0.5*np.hypot(scale_x, scale_y)`。
5. `half_height=0.5*scale_z`。
6. 删除 `radius=0.05` 和 `half_height=radius` fallback。

任一非目标 marker 无效会让整次 load 返回 `None`，不留下部分 obstacle list。

- [ ] **Step 5：保持三种状态**

- 成功 topic + 排除目标后 `[]`：有效 loaded-empty。
- topic 超时或订阅失败：`None`。
- 任一非目标 marker/TF 无效：`None`。

不得用 `if not obstacles` 把前两种合并。

- [ ] **Step 6：更新三消费者回归**

更新现有测试名和错误字符串中的 `scene_description` 为 `scene_clearance_bounds`，并明确验证：

- round-top missing/invalid -> fail closed；
- round-top loaded-empty -> 允许继续筛选；
- side missing -> 保持 warning 后跳过 corridor；
- side near/far -> 使用真实 bounds 仍按预期 reject/keep；
- safe placement missing -> 继续服从 `GRASP_PLACE_REQUIRE_SCENE`；
- safe placement loaded -> 使用同一 bounds obstacle list；
- shared loader 只订阅一次新 topic，不存在旧 topic fallback。

- [ ] **Step 7：运行 GREEN**

```bash
python3 -m pytest src/my_course_pkg/test/test_pick_place_planner.py -v
```

预期：全部 planner 测试通过，且错误文本只指向新 bounds topic。

---

## Task 5：实现全阶段“连续 3 次”资格语义

**文件：**

- 修改：`src/my_course_pkg/my_course_pkg/grasp_eval.py`
- 修改测试：`src/my_course_pkg/test/test_grasp_eval.py`
- 修改：`docs/superpowers/specs/2026-07-13-apple-stable-grasp-roadmap-design.md`
- 修改：`docs/superpowers/specs/2026-07-13-scene-clearance-bounds-and-three-run-validation-design.md`

- [ ] **Step 1：写 evaluator RED**

新增：

- `test_parse_args_defaults_to_three_trials`
- `test_parse_args_preserves_explicit_longer_trial_count`
- `test_qualification_streak_requires_three_consecutive_successes`
- `test_qualification_streak_resets_after_failure`
- `test_summarize_results_includes_qualification_line`
- `test_summarize_results_reports_pass_only_after_three_consecutive`

序列 `[True, True, False, True, True]` 的当前 streak 为 2、最大 streak 为 2，不得 PASS；`[False, True, True, True]` 最大 streak 为 3，可在显式长测报告 PASS。

- [ ] **Step 2：实现常量和统计**

新增：

```text
REQUIRED_CONSECUTIVE_SUCCESSES = 3
```

`parse_args()` 的 `--trials` 默认值使用该常量。显式 `--trials N` 仍允许 `N>3` 做压力测试。

实现明确的纯函数调用链：

```text
compute_consecutive_streaks(results)
    -> (current_streak, max_streak, passed)

summarize_results(results)
    -> 保留现有 Trials/Human-confirmed/Auto estimate 三行
    -> 调用 compute_consecutive_streaks(results)
    -> 在输出末行追加 Qualification
```

资格行格式固定为等价的：

```text
Qualification: PASS (max streak 3/3)
Qualification: NOT PASS (max streak N/3)
```

`summarize_results()` 仍只接受 `results` 并返回一个字符串，`main()` 仍只负责 `print(summarize_results(results))`；不把 CLI 或 I/O 放进 streak 纯函数。只有最大连续 streak `>=3` 才 PASS。默认 3 次批次中任何一次失败都会 NOT PASS；下一次正式资格批次必须新开一轮，从 0 开始。

- [ ] **Step 3：精确修改 Apple roadmap**

只修改资格次数，不改无关数字：

- Phase A 固定场景验收：连续 5 次 -> 连续 3 次。
- Phase G 标题、说明和记录表：五次/5 行 -> 三次/3 行。
- Apple 后续同类物体固定位置资格门：5 -> 3。
- 五个类别代表物的资格门：5 -> 3。
- 随机位置最低场景覆盖：5 -> 3。
- 最终 DoD：连续 5 次 -> 连续 3 次。

保留：

- “五类对象”“五个代表物”等类别数量；
- lift 后主动保持 `5 秒`；
- 第 5 个固定槽位/hammer 等位置编号；
- 任何几何数值中的 `5 mm`。

- [ ] **Step 4：更新 spec 状态**

把 scene-clearance design 的状态改为“用户已批准，进入实施计划/实施”，不重写已批准方案正文。

- [ ] **Step 5：运行 GREEN 和文档审计**

```bash
python3 -m pytest src/my_course_pkg/test/test_grasp_eval.py -v
```

然后人工/脚本检查 roadmap 中剩余的 `5`：每一处必须属于五类、第五槽位、5 秒 hold、5 mm 或其他非资格含义。

---

## Task 6：实施主工作区完整离线回归和 build

**工作区：** `/home/ws`。

- [ ] **Step 1：运行 simulator suite**

```bash
python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py -v
```

- [ ] **Step 2：运行 focused grasp suite**

```bash
python3 -m pytest \
  src/my_course_pkg/test/test_grasp_selector.py \
  src/my_course_pkg/test/test_pick_place_planner.py \
  src/my_course_pkg/test/test_grasp_executor_gripper_failures.py \
  src/my_course_pkg/test/test_trajectory_planner.py \
  src/my_course_pkg/test/test_grasp_eval.py -q
```

- [ ] **Step 3：build**

```bash
colcon build --packages-select my_course_pkg --symlink-install
```

仿真端不是 colcon package；它通过上述 pytest 和后续 live launch 验证。

- [ ] **Step 4：静态检查**

```powershell
git -C E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable diff --check -- \
  src/ifl_air_mujoco_sim/env/utils/scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/env/utils/populate_scene.py \
  src/ifl_air_mujoco_sim/env/mjcontrol_interface.py \
  src/ifl_air_mujoco_sim/env/ros2_interface.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py \
  src/my_course_pkg/my_course_pkg/grasp/config.py \
  src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py \
  src/my_course_pkg/test/test_pick_place_planner.py \
  src/my_course_pkg/my_course_pkg/grasp_eval.py \
  src/my_course_pkg/test/test_grasp_eval.py
```

Windows CRLF 提示可记录，但实际 whitespace error 必须修复。

---

## Task 7：live topic-only 验证

**安全限制：** 只重启/运行 simulator 并读取 topic；不运行 perception、`grasp_demo`、MoveIt plan/execute、arm 或 gripper 命令。

- [ ] **Step 1：重启 simulator 使 Python 源码生效**

使用项目当前可靠 launch 工作流。启动日志必须显示 geometry cache 成功覆盖 6 个已选物体；任何 object/geom/cache 异常都停止。

- [ ] **Step 2：检查 topic 契约**

```bash
ros2 topic type /scene_description
ros2 topic type /scene_clearance_bounds
ros2 topic info /scene_clearance_bounds --verbose
ros2 topic echo /scene_description --once
ros2 topic echo /scene_clearance_bounds --once
```

确认：

- 两个 topic 都是 `visualization_msgs/msg/MarkerArray`。
- bounds publisher QoS 是 reliable + volatile。
- 两个数组都覆盖当前 6 个 canonical object name。
- old topic 的 position/orientation 仍是 body truth。
- bounds topic 的 orientation 全 identity，scale 全 finite/positive。

- [ ] **Step 3：独立核对 hammer/banana/Apple**

以 bounds marker center/scale、同周期 `/scene_description` body pose、OBJ 顶点做独立 transform：

```text
expected_world_vertices = R_body @ body_local_vertices + p_body
expected_center/size = min/max(expected_world_vertices)
```

误差使用数值容差（建议 `<=1e-6 m`）。hammer/banana scale 必须明显不是 `[0.1,0.1,0.1]`；hammer center 必须体现 off-origin 几何中心。

- [ ] **Step 4：检查 atomic recovery**

不向运行中的仿真注入 NaN。用单元测试证明故障行为；live 只检查健康 array 每帧 marker 数恒为 6，不能观察到 1–5 个 marker 的部分帧。

- [ ] **Step 5：检查 cadence 和性能**

```bash
ros2 topic hz /scene_description
ros2 topic hz /scene_clearance_bounds
ros2 topic hz /joint_states
```

至少观察 100 个 bounds 周期。通过门：

- bounds rate `>=18 Hz`，并与配置 `20 Hz`/旧 scene rate 基本一致；
- 启动日志的一次性 100-sample AABB profile 为 p95 `<=10 ms`；
- 无持续 `loop >100 ms` 或反复 bounds error。

任一门失败：保持 A3 blocked，保存 profile/topic 输出并单独设计性能修复；不得临时改用 local-AABB 八角点、固定尺寸或其他未经审核近似。

若 p95 通过但接近 `10 ms`，将实际值和 shared-thread 风险写入交付摘要；本轮不因此现场优化或改变已批准的真实顶点算法。

- [ ] **Step 6：确认运动命令未执行**

会话记录明确写明没有调用 `grasp_demo`、arm action/service 或 gripper action。

---

## Task 8：受控同步到原工作区

**源：** `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`

**目标：** `E:\IFL\ros2-docker-workspace-vscode-plmrs`

- [ ] **Step 1：重新记录两边 status**

确认目标文件是否有实施期间的新用户改动。若同一目标行出现未知修改，停止同步并报告冲突，不猜测覆盖。

- [ ] **Step 2：逐文件应用相同语义 patch**

用 `apply_patch` 分别修改目标工作区；不使用目录级复制、不覆盖整个 dirty 文件。同步 simulator、planner、evaluator 和新增 tests。

路线图和正式设计以原工作区为文档主副本；若实施工作区后续也加入同名文档，则保持语义一致，但不要求把所有历史文档强制镜像。

- [ ] **Step 3：语义比较**

对每个相关代码/test 文件：

1. 统一 CRLF/LF 后比较文本；
2. 忽略仅行尾差异；
3. 确认函数、常量、test names 和错误策略相同；
4. 不把 `__pycache__`、outputs、logs、assets 或其他 untracked 运行产物纳入同步。

- [ ] **Step 4：在目标工作区重跑离线验证**

至少重跑 Task 6 的 simulator suite、focused grasp suite 和 build，证明同步后不是只做了文本复制。

- [ ] **Step 5：更新两个 handoff**

简短记录：

- 新 topic 契约和几何来源；
- planner 三消费者迁移；
- 测试/build/live topic-only 结果；
- 性能数据；
- 全局 3 次规则；
- Apple A3 是否已解除前置阻塞；
- 下一步仍应是 A3 plan-only，而不是直接机械臂运动。

---

## Task 9：最终审查与交付

- [ ] **Step 1：需求映射检查**

逐条对照正式设计第 4–10 节和本计划 Definition of Done，提供每条对应代码、测试或 live 证据。

- [ ] **Step 2：fallback 扫描**

检查相关 planner/publisher 路径，确保不存在：

- mesh `0.1 m` clearance fallback；
- zero scale -> `0.05 m` radius；
- invalid half-height -> radius；
- bounds topic 失败后回读 `/scene_description`；
- 单物体失败后 `continue` 发布部分 array。

旧 `/scene_description` 的可视化 mesh cube fallback 可以保留，因为它不再承担 clearance，且本任务要求保持旧 topic 行为；必须用注释防止未来误用。

- [ ] **Step 3：安全边界检查**

确认整个实现会话没有进入 A3/MoveIt/arm/gripper。只有用户审阅实施结果并明确批准后，下一会话才能运行 Apple A3 plan-only。

- [ ] **Step 4：提交交付摘要**

摘要只报告：修改结果、测试/build 数量、live topic/性能证据、两个工作区同步状态、剩余风险和下一安全动作。工作区含无关 dirty changes 时，不自动 stage 或 commit。

---

## 3. 实施风险与止损点

1. **MuJoCo XML transform 语义错误**：synthetic quat/euler/scale tests 与真实 YCB reference tests 必须先过，不能靠视觉判断。
2. **collision mesh 顶点过多导致 publish 变慢**：先按批准方案做真实顶点 transform；若 p95 超过 10 ms，停止 A3 并单独审查优化，不现场换近似。
3. **旧 topic 被意外改语义**：publisher contract test 和 live 双 topic 对比是强制项。
4. **planner 把 `[]` 当 `None`**：round-top 只目标场景测试必须保留。
5. **side 行为被顺带收紧**：本轮只换数据源；missing scene 仍按原策略跳过 corridor。
6. **safe placement 偷偷回读旧 topic**：禁止 fallback，测试订阅 topic 和缺失策略。
7. **“3 次”误改“5 秒/五类/5 mm”**：文档更新后逐个剩余数字审计。
8. **跨工作区覆盖用户改动**：逐文件 semantic patch；发现同区冲突立即停止。

## 4. 完成后的下一步

本计划全部 DoD 通过、用户审阅实施结果后，下一步才是 Apple Phase A3 plan-only：生成并检查 pregrasp -> grasp 垂直接近通道、MoveIt 可达性和 clearance 诊断，但仍不执行机械臂运动。A3 通过后再依次进入 `GRASP_DEBUG_STOP_AT_PREGRASP`、`...AT_GRASP`、`...AFTER_CLOSE`、`...AFTER_LIFT`、`...BEFORE_RELEASE` 和完整 3 次闭环。
