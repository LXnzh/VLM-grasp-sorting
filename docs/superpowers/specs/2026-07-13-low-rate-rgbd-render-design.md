# 低频 RGB-D 渲染恢复状态循环频率设计

**日期：** 2026-07-13

**状态：** 基于 Stage 1/2 实测提出；受控 `0.2 Hz` 配置实验已获用户批准，
尚未修改配置或重启仿真

**目标：** 保留单路 `camera_orbbec` 的 1280x720 RGB-D 质量与现有单线程
MuJoCo 数据所有权，只降低相机生成帧的时间频率，使 physics、joint states、
scene description 和 clearance bounds 恢复到 `>=18 Hz`。在该配置方案实测
失败前，不引入线程、锁、`mjData` 快照或 perception 接口改造。

## 1. Stage 1/2 实测结论

### Stage 1：单路 1280x720

- 移除 perception 未使用的 `camera_third_person`。
- 只保留 `camera_orbbec` RGB+depth，关闭 pointcloud。
- 三个状态 topic 从约 `1.48 Hz` 提升到约 `3.43 Hz`。
- 单路相机仍占约 `212-293 ms`/帧。

### Stage 2：单路 640x480

- 三个状态 topic 约为 `3.77 Hz`，远低于 `18 Hz`。
- 相机仍占约 `195-251 ms`/帧，分辨率降低收益很小。
- 该配置已从工作区恢复，不作为接受状态保留。

此外，`rgbd_file_save.py::_init_camera_intrinsic()` 当前把内参固定为
1280x720。直接使用 640x480 不只是需要重新验证 A1，还会先造成内参与图像
尺寸不一致。因此当前阶段应保留 1280x720，不能把 640x480 当作无代码代价的
配置优化。

## 2. 新发现的最低复杂度路径

pipeline 的 `capture_rgbd_once()` 只等待第一组同步 RGB-D，然后立即保存并
销毁订阅节点；等待超时为 `30 s`。它不需要 20 fps 连续视频。

`ros2_interface.py` 已经把状态发布周期与相机周期分开：

- 状态发布：`ros_publish_hz: 20`；
- 相机调度：`_cam_period = 1 / render_fps`；
- `_sim_loop` 只在 `now >= next_cam_t` 时渲染。

虽然 `MuJoCoInterface` 内部会把传入的 `render_fps` 取整并限制到至少 1，
但对应 `_render_fps/_steps_per_render` 字段目前没有参与 ROS `_sim_loop` 的
相机调度。实际调度使用 `UR10eRos2Interface._cam_fps` 的浮点值，因此
`render_fps: 0.2` 可表达每 5 秒生成一组 RGB-D。

## 3. 方案比较

### 方案 A：0.2 Hz 连续 RGB-D（推荐）

保留：

```yaml
camera_names: ["camera_orbbec"]
camera_size: [1280, 720]
enable_pointcloud_camera1: false
enable_depth_camera1: true
```

只修改：

```yaml
render_fps: 0.2
```

已测单帧阻塞约 0.20-0.29 秒。每 5 秒阻塞一次，理论上每秒损失不到一轮
20 Hz 状态发布，预期平均状态频率约 19 Hz，同时 pipeline 最坏约 5.3 秒
拿到第一帧，仍显著低于 30 秒超时。

优点：不改线程模型、不并发访问 `mjData`、不改图像尺寸/内参、不改 pipeline
接口。缺点：相机不再适合实时视频观察；但当前 perception 只取一帧。

### 方案 B：按需 RGB-D 服务

默认不渲染；pipeline 先请求一次 capture service，仿真线程在安全点生成并
发布一组 RGB-D。

优点：平时完全没有相机渲染负载。缺点：需要新增服务、修改 pipeline、处理
请求超时与重复请求，改动面明显大于当前需求。

### 方案 C：独立 render-data 线程

physics 线程独占 live `mjData`，每次 step 后生成不可变状态快照；相机线程
拥有独立 `mujoco.MjData` 与 renderer，把快照复制到 render-data、执行
`mj_forward()` 后渲染。

优点：可同时维持高频 physics/state 与较高相机帧率。缺点：必须证明共享
只读 `MjModel`、独立 `MjData`、OpenGL 上下文生命周期和 reset 的线程安全，
实现与测试成本最高。

不接受“publisher 线程以 20 Hz 重发低频旧状态”的伪解耦方案。

## 4. 方案 A 实施范围

第一轮只在 active `-grasp-stable` 工作区修改：

1. `base_env.yaml`：单路 `camera_orbbec`、1280x720、`render_fps: 0.2`。
2. 现有默认配置测试：锁定上述相机与 depth/pointcloud 映射。
3. 不修改 `ros2_interface.py`、`mjcontrol_interface.py` 或 perception 代码。

当前 active 工作区已经满足单路 `camera_orbbec`、1280x720、depth 开启、
pointcloud 关闭的前置配置，因此本轮生产配置的唯一变化是
`render_fps: 20 -> 0.2`。

## 5. 验证门

离线验证：

- 配置测试先 RED 后 GREEN；
- 65 项 simulator/scene-clearance 聚焦回归通过；
- `git diff --check` 通过。

live topic-only 验证：

按项目级“两次连续成功”政策执行 2 次独立仿真启动与测量。每次都必须从新启动的
active 配置开始，并分别满足以下全部条件；任一次失败都使 cadence 连续计数归零：

1. 只存在 `camera_orbbec` RGB/depth publisher。
2. RGB 为 1280x720 `bgr8`，depth 为 1280x720 `32FC1`。
3. 观察至少 35 秒，获得至少 6 组 RGB-D 帧；颜色与深度时间戳匹配。
4. 同一运行中测量至少 30 秒：
   - `/scene_clearance_bounds >=18 Hz`；
   - `/scene_description >=18 Hz`；
   - `/joint_states >=18 Hz`。
5. bounds 仍包含 6 个完整对象，QoS 为 reliable/volatile，无构建或发布错误。
6. 记录 `loop/phys/js/cam` profile，确认没有通过重发旧快照伪造频率。

若任一状态 topic 未达到 `18 Hz`，立即停止，保存 topic/profile 证据，并把
`render_fps` 恢复到实验前的 `20`。不继续把相机降到更低频率，也不在同一
实验中运行 `grasp_demo`；转而评审方案 B、C，或单独设计以功能性控制健康门
替换固定 `18 Hz` 数值门。

## 6. 通过后的 A1/A3 顺序

方案 A live 门全部通过后，按项目级“两次连续成功”政策执行：

1. 执行 2 次独立 reset + fresh perception A1 只读验证。
2. 每次 pipeline 必须在 30 秒内获取新的同步 RGB-D。
3. 自动 mask 与 geometry-center `<10 mm` 门全部通过。
4. 不使用 `FOUNDATIONPOSE_MASK_INDEX`。
5. 两次 A1 通过后，再执行 2 次独立 Apple A3 `grasp_plan_only`；每次必须
   使用当前 reset 的 fresh perception，报告 `exact_bounds_box_2p5d`、至少一个
   MoveIt 可达候选、成功恢复 `planonly=False`，且无执行运动。
6. 两次 A3 全部通过后，低频配置才具备进入 Phase B 的感知/规划资格。

在两次 live topic 门和两次 A1 通过前，不运行 A3。两次 A3 通过前不运行
`grasp_demo`、机械臂或夹爪。

## 7. 固定频率门失败后的边界

用户决定：如果 `0.2 Hz` 配置仍无法达到固定 `>=18 Hz` 门槛，后续可以取消
该固定数值要求并继续推进。但取消数值门不等于取消控制健康检查。

该 fallback 必须作为单独设计，使用实测数据定义至少以下功能性条件：

- `/joint_states` 和控制状态的最大消息间隔与数据新鲜度；
- 相机单帧阻塞期间的最大 physics/control stall；
- MoveIt 运动和后续 Cartesian 收敛所允许的状态抖动与超时；
- 失败时在 approach 或 gripper 命令前停止的 fail-closed 行为。

低频配置实验失败后不得立即运行 Phase B。必须先恢复配置、保存证据、批准并
验证新的功能性健康门设计；禁止完全无替代门槛地执行机械臂运动。
