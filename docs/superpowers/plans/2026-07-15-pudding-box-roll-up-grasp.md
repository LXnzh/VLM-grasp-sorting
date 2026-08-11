# Pudding Box 滚边翘起抓取实施计划

**目标：** 只为规范化精确名称 `pudding_box` 实现固定方向的“预夹—远侧底边滚起—厚度锁紧—30 mm 试抬—提升保持”，并用自动化隔离测试证明其他物品的候选、配置、夹爪命令和执行路径不变。

**设计依据：**
`docs/superpowers/specs/2026-07-15-pudding-box-roll-up-grasp-design.md`
（最终设计审阅提交 `8620d23`）

## 实施总原则

1. 运行代码只在规范化结果精确等于 `pudding_box` 时构造；非 Pudding 不创建 monitor、MoveIt 校验器、URDF 包络或专属执行器。
2. 第一阶段只允许规划世界 `-X` 接近、`+X` 滚动；固定方向失败立即停止，不换侧、不回退旧抓取库。
3. 先完成无机械臂运动的几何/配置/路由硬门，再逐级解锁预位姿、开爪接近、预夹、10°、20°、回滚、25–30°、闭合和提升。
4. 不修改 `grasp_selector.py`、`trajectory_planner.py` 或通用 `executor.py`。若实施中确实需要修改其中任何一个文件，暂停本计划并先补充设计审阅。
5. 不自动运行仿真或机器人运动。所有阶段 4–13 的首次动作都由用户明确启动并现场观察。
6. 当前工作树已有大量非 Pudding 修改；每次暂存前必须按精确路径核对，禁止把无关改动纳入 Pudding 提交。

## Task 1：冻结非 Pudding 基线和专属配置边界

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Add: `src/my_course_pkg/test/test_pudding_config_isolation.py`
- Add: `src/my_course_pkg/test/data/pudding_non_target_config_baseline.json`

1. 先记录当前分支、全部 dirty 路径和将被 Pudding 修改的重叠文件 SHA/差异摘要。当前已验证的 pear/通用边界修改仍未全部提交，不能直接切到干净 HEAD worktree 后丢失它们。
2. 只有当重叠的现有修改已经被提交或导出为可复现补丁、且用户明确同意时，才创建 Pudding 专属 branch/worktree；否则留在当前工作树，用精确文件/精确 hunk 暂存和基线测试隔离，绝不 stash、reset 或覆盖现有改动。
3. 在修改 `config.py` 前，用清理全部 `GRASP_*` 环境变量的独立 Python 子进程导出当前工作树的非 Pudding 抓取配置快照。
4. 基线覆盖所有 `VERTICAL_*`、`SIDE_*`、`ROUND_TOP_*`、`CENTERED_*`、`TOP_DOWN_*`、共享 gripper 默认值，以及对象 profile/category/geometry 映射；记录生成脚本版本和 JSON SHA256。
5. 只增加 `PUDDING_*` 常量，包含：场景数量/桌面期望值、bounds 新鲜度和尺寸窗口、固定方向、切向偏移、预接近距离、65 mm 桌边门限、三态净空、支撑间隙、闭合轴误差、预夹/闭合 effort、预夹稳定窗口、滚动 checkpoint、模型残差、倾斜验证、试抬、恢复尝试次数和阶段解锁配置。
6. 添加 `PUDDING_EXECUTION_STAGE`，默认值为 `validate_only`。它只表示正常前进路径允许到达的最高状态，允许值严格枚举为：
   `validate_only`、`calibrate`、`plan_only`、`prepose`、`open_support`、
   `preclamp`、`roll_10`、`roll_20`、`tilt_30`、
   `full_close`、`test_lift`、`full_lift`。
7. 单独增加 `PUDDING_TEST_ACTION`，默认 `none`，第一阶段唯一非默认值为 `rollback_from_20`；它是独立验证命令，不是前进 stage。
8. 测试断言新增常量全部以 `PUDDING_` 开头，非 Pudding 基线逐项不变；环境变量非法、NaN、负值、门限顺序错误、未知 stage/action 或不兼容组合时导入失败。
9. 此任务不改变任何对象映射、共享 profile、共享阈值或共享 gripper 默认值。

## Task 2：先扩展夹爪 effort 接口并证明默认行为等价

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/gripper_control.py`
- Add: `src/my_course_pkg/test/test_gripper_control_effort.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`（只增加回归断言）

1. 先添加失败测试：现有 `send_gripper_command(position)` 在 action 和 topic/native 两条路径中仍发送当前 `GRIPPER_EFFORT`。
2. 把接口最小扩展为 `send_gripper_command(position, effort=None)`；`None` 必须在函数内部解析为原共享值。
3. 把解析后的同一个 effort 显式传给 action goal 的 `max_effort` 和 `ArmApi2Client.send_gripper_command` 的第二参数。
4. 保持开爪重试次数、acceptance 判定、日志字段、返回数据结构和所有调用点不变。
5. 基线测试先确认当前 `GRIPPER_EFFORT` 默认值确为 140 N；再添加 Pudding 显式 50 N/140 N 测试、NaN/负 effort 失败测试，以及 action/topic 请求字段逐项回归。Pudding 的 140 N 不能重写共享默认值。
6. 运行现有 gripper/executor 全套测试；此任务不修改通用 executor 的调用代码。

## Task 3：实现精确路由谓词和通用 planner 防回退保护

**Files:**

- Add: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_planner.py`
- Add: `src/my_course_pkg/test/test_pudding_routing.py`
- Modify: `src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
- Modify: `src/my_course_pkg/test/test_pick_place_planner.py`

1. 在 Pudding 模块中增加唯一公开路由函数 `is_pudding_roll_target(name)`；其他数学函数均以下划线开头保持私有。
2. 规范化规则与设计 §5 一致：trim、lower、空格/连字符转下划线、移除可选 YCB 数字前缀，然后只比较 `pudding_box`。
3. 参数化测试 canonical 等价写法应进入；`pudding_boxes`、前后缀、随机近似字符串和空值不得进入。
4. 从配置中的完整对象映射动态枚举所有非 Pudding 名称，证明它们都返回 false。
5. 在 `plan_pick_place_candidates_from_perception()` 读取对象名后的第一时间加入调用层保护：若通用 planner 被直接错误调用且目标是精确 Pudding，立即 raise 安全异常，发生在 FoundationPose、旧抓取库和 vertical 居中之前。正常重定向只由 Task 12 的 `demo.py` 顶层分流完成；这里不重定向，也不由 try/catch 回退旧路径。
6. 对所有非 Pudding 固定输入比较 planner 调用顺序、结果矩阵、步骤名称和 debug 字段，要求与修改前一致。

## Task 4：实现线程安全的 Pudding 场景监控器

**Files:**

- Add: `src/my_course_pkg/my_course_pkg/grasp/pudding_scene_monitor.py`
- Add: `src/my_course_pkg/test/test_pudding_scene_monitor.py`

1. 定义不可变 `PuddingSceneBound` 和 `PuddingSceneSnapshot` dataclass，保存接收单调时间、规范化名称集合、目标 bounds、非目标 bounds、`T_world_bounds`、half-extents 和 `table_z_runtime`。
2. 监控器只在实例中保存 subscription、`threading.Lock` 和当前快照；禁止模块级可变缓存。
3. 回调先在局部变量中验证完整 MarkerArray、唯一名称、有限变换、正尺寸和 TF，再深复制并在锁内一次替换快照引用。
4. 从初始 6 个 bounds 底面 Z 的中位数计算 `table_z_runtime`，要求底面 spread ≤ 5 mm 且与 0.890 m 期望值差 ≤ 10 mm。
5. `snapshot(max_age_sec)` 在锁内读取引用，在锁外返回不可变对象；目标消失、重复、名称集合变化、尺寸异常、TF 失败和年龄超限都失败关闭。
6. 测试并发读写不会产生混合代际字段；测试非 Pudding 路由从未构造监控器、没有新增订阅。
7. 提供显式 `close()` 销毁 subscription，并测试成功、异常和 Ctrl-C 清理路径。

## Task 5：建立实际 URDF 三态完整包络硬门

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_planner.py`
- Add: `src/my_course_pkg/test/test_pudding_gripper_envelope.py`
- Add: `src/my_course_pkg/test/data/pudding_envelope_fixture/`（最小 URDF 与 STL fixture）
- Modify: `src/my_course_pkg/package.xml`

1. 只在 Pudding 分支通过 ROS 参数客户端读取运行中的扩展 `robot_description`，记录来源节点和 SHA256；缺失时不使用仓库静态 xacro猜测，直接失败。
2. 使用已安装的 `urdf_parser_py` 解析 collision 元素，用 `ament_index_python` 解析 `package://` 路径；在 `package.xml` 中显式声明 `urdfdom_py`、`ament_index_python` 和 `rcl_interfaces` 依赖。
3. 支持 box、cylinder、sphere 和 binary/ASCII STL collision geometry，应用 mesh scale 和 collision origin。遇到 DAE、缺失 mesh、零尺寸或无法解析的 Pudding 相关 link 时失败关闭。
4. 从配置的末端执行器根 link 遍历实际 URDF 子树，必须包含两个 pad/finger/knuckle、gripper base/适配器、双相机支架和 Orbbec collision；缺少任何必需 link 时失败。
5. 在安全高位的 `calibrate` 阶段分别发送开爪、若干中间命令和闭合命令，读取实际 joint/TF 反馈，把每个 collision 顶点转换成 TCP 相对点，形成开爪/预夹/闭合三态包络和“命令值 → pad 间距”表。
6. 包络只存在于本次 Pudding 运行实例并写入日志；不修改 URDF、MoveIt collision 或共享机器人状态。
7. 离线 fixture 测试 mesh scale、collision origin、TF 链、三态差异、最低 Z、桌边/障碍物距离、URDF SHA 变化和不支持格式失败。
8. **硬门 A：** 在不移动机械臂的候选几何报告中，证明开爪完整包络能通过 612.6 mm 接近侧；该数值对应“仿真 `+Y` → 规划 `-X`”。888.9 mm 对应“仿真 `-X` → 规划 `-Y`”且 Tuna 位于该侧，不能替换。硬门失败则停止后续运动验证。
9. **硬门 B：** 证明 `[0,3] mm` 支撑间隙与全部相关 collision 点 ≥ 5 mm 桌面净空能够同时成立；否则停止，不降低门限。

## Task 6：实现纯滚动几何、候选和完整净空筛选

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_planner.py`
- Add: `src/my_course_pkg/test/test_pudding_roll_planner.py`

1. 定义不可变 `PuddingRollCandidate`、`PuddingRollCheckpoint`、`PuddingRollPlan` 和失败原因结构；计划数据不复用通用 `PickPlacePlan`。第一阶段固定 `-X/+X`、最多 6 候选是最终 spec 已批准的当前固定布局范围，不实现或预留四侧自动回退。
2. 从初始 `T_world_bounds + half-extents` 求 `r`、`L`、`H`、`p_far`、`p_near` 和 `tangent`；验证规划 `+X` 对齐误差 ≤ 5°、旋转符号抬高近侧角和远侧轴保持。
3. 只生成 `-X` 接近、切向 `-20/0/+20 mm`、腕部 0/180° 的最多 6 个候选。
4. 预位姿严格沿工具接近轴反向 100 mm；65 mm 桌边粗门按 `24 + 10 + 30 mm` 记录分项来源。
5. 为 0–30° 生成每 5° validation checkpoint；必要时插入更细 trajectory waypoint，使平移 ≤ 10 mm、旋转 ≤ 5°。
6. 在每个状态使用正确的开爪/预夹/闭合包络，验证桌面 ≥ 5 mm、非目标 ≥ 30 mm、桌边和相机支架净空。
7. 输出开爪支撑 gap、最低 Z、所有障碍物最小距离、预测中心轨迹、TCP 轨迹和理论 AABB 高度。
8. 单元测试远侧轴不动、下指沿圆弧、正负旋转、坐标轴交换、错误 yaw、固定方向不可用和所有门限边界。

## Task 7：增加 MoveIt IK/状态有效性和预位姿纯规划

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_planner.py`
- Modify: `src/my_course_pkg/package.xml`
- Add: `src/my_course_pkg/test/test_pudding_moveit_validation.py`

1. 只在 Pudding planner 实例中创建 `/compute_ik` 和 `/check_state_validity` 客户端；显式声明 `moveit_msgs` 依赖。
2. 对每个候选按 prepose → open support → 全部 trajectory waypoint 顺序计算 IK，后一个状态以上一个关节解为 seed。
3. 把对应 gripper joint 状态写入 RobotState，调用 state validity 检查完整机器人、桌面和场景碰撞；任何缺失 joint、超时或 collision 都拒绝候选。
4. 对 prepose 再使用 `ArmApi2Client.set_planonly(True)` + `move_to_pose()` 验证从当前机器人状态可规划到达，并在 `finally` 中恢复 `planonly=False`。
5. plan-only 清理失败时锁死 Pudding 后续动作并给出恢复命令，不尝试 Cartesian fallback。
6. Mock 测试 IK seed 传递、三态 gripper joints、服务超时、碰撞拒绝、候选顺序、plan-only 恢复和固定方向全失败。
7. **硬门 C：** 6 个固定方向候选至少有一个通过全部几何、IK、状态有效性和 prepose plan-only；否则停止，不换侧。

## Task 8：实现专属状态机骨架和阶段解锁

**Files:**

- Add: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_executor.py`
- Add: `src/my_course_pkg/test/test_pudding_roll_executor.py`

1. 定义显式状态枚举、允许转移表、每状态输入/输出和统一 `PuddingRollExecutionError`。
2. 执行器通过构造函数注入 motion adapter、gripper controller、scene monitor、plan 和时钟；单元测试不需要 ROS 或实际动作。
3. `PUDDING_EXECUTION_STAGE` 是正常前进路径的单向上限；另用运行时 `highest_forward_state` 记录实际达到的最高状态。达到配置上限立即保持/返回报告，绝不能自动进入下一阶段。
4. 默认 `validate_only` 只加载快照、配置和离线几何报告，不发送 arm/gripper 命令。
5. 每个状态进入前重复检查名称、bounds 年龄、场景集合和阶段上限；失败阻止所有后续状态。
6. 安全恢复不受前进 stage 上限阻止，可随时沿已记录路径向后执行；`PUDDING_TEST_ACTION=rollback_from_20` 只允许与 `PUDDING_EXECUTION_STAGE=roll_20` 组合，到达 20° 后调用同一个恢复方法，完成即结束会话并要求新 reset。
7. 测试完整顺序、跳步拒绝、每个阶段停点、test action 组合、异常清理、bounds 丢失保持、可恢复/不可恢复失败分类。

## Task 9：实现标定、预位姿、开爪支撑和预夹

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_executor.py`
- Modify: `src/my_course_pkg/test/test_pudding_roll_executor.py`

1. `calibrate` 只允许在初始安全高位运行；完成三态包络/命令间距表后恢复开爪并停住。
2. `prepose` 使用 MoveIt 到达已经 plan-only 通过的预位姿；失败尝试同固定方向下一个已验证候选。
3. `open_support` 用低速 Cartesian trajectory waypoint 到达支撑姿态；每步检查 TCP 收敛、开爪包络、bounds 新鲜度和 `[0,3] mm` gap。
4. `preclamp` 显式发送 50 N 和标定求得的位置；验证 action/topic 结果不为空夹、不过深、不过早。
5. 预夹前记录绝对平放基线 `c_flat/h_flat`；等待 ≥ 0.3 s 和 ≥ 3 个新鲜快照，平面位移、中心 Z、AABB 高度变化分别 ≤ 5 mm 后，再记录滚动模型基线 `c0/h0`。日志同时输出两组基线和差值，恢复相对平放基线判定，滚动模型相对预夹后基线计算。
6. 预夹失败但盒体仍平放时只开爪并沿已执行路径退回；位置不可信时保持等待人工。
7. 测试命令参数、快照窗口、边界值、空夹、夹偏、阶段停点和 0° 恢复。

## Task 10：实现 5° 滚动 checkpoint、倾斜验证和受控回滚

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_executor.py`
- Modify: `src/my_course_pkg/test/test_pudding_roll_executor.py`

1. checkpoint 严格为 5/10/15/20/25/30°；每个 trajectory waypoint 后验证 TCP，checkpoint 停稳后才读取 bounds。
2. 检查高度总体增加、单步回退 ≤ 5 mm、连续两步不增高失败、平面总位移 ≤ 40 mm、模型 XY/Z 残差和 TCP 相对中心残差。
3. 记录 0/5/10/15/20/25/30° 全部 checkpoint 的实际角度—AABB 高度曲线；同一次运行不允许据此放宽阈值。
4. `VERIFY_TILT` 必须满足 AABB 高度 ≥ 85 mm、中心 Z 增量 ≥ 20 mm、平面位移 ≤ 40 mm 和新鲜 bounds。
5. `rollback_from_20` test action 在独立试验中从 20° 按 5° 反向 checkpoint 回到 0°；它调用与真实失败恢复相同的方法，不是正常状态。每个反向 checkpoint 最多 3 次尝试，第 3 次失败后保持预夹等待人工。
6. 命令回到 0° 后要求连续 3 快照满足初始高度/中心/底面/平面窗口，之后才允许开爪。
7. 测试理想轨迹、噪声容忍、滑动、未翘起、残差超限、回滚重试上限和平放验证失败保持。

## Task 11：实现完全闭合、30 mm 试抬、提升保持和失败落回

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pudding_roll_executor.py`
- Modify: `src/my_course_pkg/test/test_pudding_roll_executor.py`

1. 只有 `VERIFY_TILT` 通过且机械臂停稳后，显式发送 140 N 完全闭合。
2. 用标定表判断接触开度是否对应约 39.6 mm 厚度；空夹到底或异常早停失败。
3. 试抬用 5–10 mm waypoint 累计 30 mm；要求目标中心跟随 ≥ 20 mm、相对 TCP 平面漂移 ≤ 15 mm、bounds/夹爪反馈持续有效。
4. 试抬成功后提升到现有正常高度并保持 5 s；不调用通用 `lift_return`、不自动张开。
5. 恢复降低在远离桌面时 ≤ 10 mm/步，预计支撑面上方 6 mm 内 ≤ 3 mm/步；检测目标底面和实际中心下降量，桌面承重后停止压低。
6. 完全闭合后保持闭合命令回滚；只有平放稳定验证通过才张开。bounds 不可信时保持等待人工。
7. 测试 close-before-tilt 禁止、厚度接触、试抬成功/掉落、低速落桌、提前承重、不可恢复保持和绝不自动释放。

## Task 12：接入 demo 精确分流并证明非 Pudding 调用顺序不变

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/demo.py`
- Modify: `src/my_course_pkg/test/test_pudding_routing.py`
- Modify: `src/my_course_pkg/test/test_grasp_plan_only.py`（只增加路由回归）

1. `GraspDemoNode.run()` 在任何 controller mode/gripper 命令和通用 planner 前读取选择对象并调用唯一路由谓词。
2. 精确 Pudding 才重定向并延迟构造 monitor、专属 planner 和专属 executor；运行结束无论成功/失败都关闭 monitor。该路径不依赖捕获通用 planner 的保护异常。
3. 非 Pudding 继续按原顺序执行：进入 servo → 读取 EE → 通用 planner → gripper ready → 通用 executor → return home。
4. Pudding 路径不调用通用 planner、通用 `execute_first_reachable_plan` 或通用 return/release。
5. 默认 `validate_only` 的 Pudding 调用只打印验证报告并退出，不移动机械臂或夹爪。
6. 对配置中的全部非 Pudding 对象做 mock/spy 测试，既捕获每次函数调用的完整参数值和调用顺序，也比较结构化输出：候选矩阵、排序、步骤名称、夹爪目标、debug 字段和返回/释放行为；特别覆盖 `gelatin_box`、`foam_brick`、`rubiks_cube`、`tuna_fish_can` 和 `pear`。

## Task 13：L1 离线回归、构建和静态隔离审计

1. L1 不要求运行 ROS graph、MoveIt 或 MuJoCo，包含 config baseline、名称路由、纯滚动数学、STL/URDF fixture 包络、mock monitor 并发、mock MoveIt validator、mock executor/state machine 和 gripper effort 单元测试。
2. 依赖可用时可在主机快速运行 L1；权威结果在已 source 的 ROS 2 容器内运行，但仍不启动任何 ROS 节点或仿真。
3. 运行现有完整 grasp-selector、pick-place-planner、trajectory-planner、executor/gripper 和 plan-only mock 测试。
4. 在清理 `GRASP_*` 环境的子进程中重新比较非 Pudding 配置基线和全部对象路由。
5. 编译全部新增/修改 Python 文件，并运行 fatal flake8：`E9,F63,F7,F82`。
6. 在 ROS 2 容器内运行：

   ```text
   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q src/my_course_pkg/test
   colcon build --symlink-install --packages-select my_course_pkg
   ```

7. 从安装空间导入 Pudding 三个模块、专属默认值、路由谓词和 optional effort 接口。
8. 核对 host/container SHA；核对 `git diff` 不包含 `grasp_selector.py`、`trajectory_planner.py`、通用 `executor.py` 或任何非 Pudding 阈值修改。
9. L1 只运行测试和构建，不启动 MuJoCo、不发送 arm/gripper 命令。

## Task 14：L2 在线集成与按硬门顺序的人工分阶段验证

1. L2a 是在线但无机械臂运动的集成验证：用户启动现有 ROS 2/MoveIt/MuJoCo 栈，测试只订阅 bounds、读取 robot_description、查询 IK/state validity 和发送受 `planonly=True` 保护的规划请求。
2. **标定前只读输入报告：** 输出 bounds、坐标映射、`p_far/tangent`、6 个候选、robot_description/mesh SHA 和尚未标定的项目清单；此时不得声称已经取得三态实际包络，也不得判定硬门 A/B 通过。
3. **安全高位夹爪标定：** 这是 L2b 的第一个显式动作，只动夹爪，记录 robot_description SHA、命令/间距表和开爪/预夹/闭合三态实际包络。
4. **标定后在线几何报告：** 用第 3 步实测三态包络重新输出最低 Z、桌边/障碍物净空和支撑间隙；硬门 A/B 必须通过。
5. **MoveIt plan-only：** 验证至少一个固定方向候选的 prepose 和全部 IK/state validity；硬门 C 必须通过。
6. **`prepose`：** 只移动到预位姿并停住。
7. **`open_support`：** 自动报告最低 collision Z 与 `table_z_runtime` 差值 ≥ 5 mm，操作员检查无异常接触。
8. **`preclamp`：** 只预夹，要求三项 5 mm 稳定门限通过，然后人工重置场景。
9. **`roll_10`：** 滚到 10° 停住并检查曲线/残差。
10. **`roll_20`：** 滚到 20° 停住并检查。
11. **`PUDDING_TEST_ACTION=rollback_from_20`：** 在 `roll_20` 会话中调用真实恢复方法回滚并证明平放后才开爪；完成即结束会话并 reset，不能继续解锁 `tilt_30`。
12. **`tilt_30`：** 新 reset 后滚到 25–30°，通过 `VERIFY_TILT` 后停住。
13. **`full_close`：** 新 reset 后闭合但不抬升，确认厚度接触。
14. **`test_lift`：** 新 reset 后完成 30 mm 试抬并保持。
15. **`full_lift`：** 新 reset 后完整提升并保持，不自动释放。
16. 每一级只有前一级日志、数值和截图均通过才解锁；失败后不在同一扰动场景继续试下一级。

## Task 15：最终验收和交接

**Files:**

- Modify: `docs/agent_handoff.md`

1. 完成三次独立 reset → 感知 → 规划 → 30 mm 试抬连续成功。
2. 随后完成三次独立 reset → 感知 → 规划 → 完整提升保持连续成功。
3. 每次记录固定方向、接触偏移、实际角度—高度曲线、预夹位移、模型残差、闭合开度、试抬跟随和全部恢复结果。
4. 再运行一次非 Pudding 完整回归并比较配置/路由/请求字段基线。
5. 更新 handoff：实现文件、测试数量、构建状态、三次试抬/提升结果、已知限制和下一步。
6. 运行代码和测试在完成分阶段验证前保持未提交；设计和实施计划文档可以单独提交。最终是否提交运行代码由用户在验收后决定。

## 明确停止条件

以下任一条件出现时停止，不通过调大共享阈值或切换其他物品路径解决：

- 实际 URDF 完整包络不能同时满足 612.6 mm 接近空间、5 mm 桌面净空和 `[0,3] mm` 支撑间隙；
- 当前固定槽位、相对 `base_link` 朝向或规划/仿真坐标映射发生变化；此时重新做设计和全部硬门，不扩展到四侧自动尝试；
- 固定 `-X/+X` 方向无候选通过 IK、state validity 或 prepose plan-only；
- 预夹把盒体推动超过 5 mm 或导致中心 Z/AABB 高度变化超过 5 mm；
- 滚动连续两步不增高、模型残差超限或平面位移超过 40 mm；
- 30 mm 试抬不跟随、bounds 失真或夹爪反馈丢失；
- 任何非 Pudding 配置、候选、请求字段、调用顺序或测试结果发生变化；
- 实施需要修改 `grasp_selector.py`、`trajectory_planner.py` 或通用 `executor.py`。
