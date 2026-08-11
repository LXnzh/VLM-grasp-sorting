# 相机渲染减负与场景状态发布频率恢复设计

**日期：** 2026-07-13

**状态：** 对话方案已批准；等待用户审核本书面版本后实施

**目标：** 在不引入并发读取 MuJoCo `mjData`、不降低安全边界质量、
不执行机械臂或夹爪运动的前提下，先移除 perception 未使用的相机渲染，
使 `/scene_clearance_bounds`、`/scene_description` 和 `/joint_states` 达到
既定的 `>=18 Hz` 门槛。只有第一阶段仍不达标时才降低主相机分辨率；
只有前两阶段均失败时才另行设计不可变状态快照方案。

## 1. 已验证的代码事实

1. `my_course_pkg/rgbd_file_save.py` 中的 perception RGB-D 节点只订阅：
   - `/camera_orbbec/color/image_raw`
   - `/camera_orbbec/depth/image_raw`
2. 当前 `base_env.yaml` 配置为：
   - camera1：`camera_third_person`
   - camera2：`camera_orbbec`
   - 两路相机都启用 RGB、depth 和 pointcloud 输入所需的 depth 渲染。
3. `ros2_interface.py::_sim_loop()` 在同一个线程中依次执行：
   - 控制目标更新；
   - `mj_step()`；
   - joint/scene/clearance 发布；
   - 两路相机 RGB/depth 渲染。
4. 当前软件渲染环境为 `llvmpipe`。live profile 显示相机渲染通常占用
   约 `470-810 ms`，而 AABB 计算 100 次样本的 median 为 `2.175 ms`、
   p95 为 `4.756 ms`。
5. 三个同循环 topic 均约为 `1.48 Hz`，因此瓶颈来自相机渲染阻塞，
   不是 AABB 计算。

## 2. 第一阶段：只关闭未使用相机

第一阶段只修改相机配置，不修改线程模型、MuJoCo 数据访问方式、planner
或 perception 代码。

配置调整：

```yaml
camera_names: ["camera_orbbec"]
camera_size: [1280, 720]
render_fps: 20
enable_pointcloud_camera1: false
enable_depth_camera1: true
enable_pointcloud_camera2: false
enable_depth_camera2: false
```

这里 `camera_orbbec` 从原 camera2 变为 camera1，因此 depth 开关必须同步
重映射到 camera1。仅关闭原 camera1 的 depth/pointcloud 开关是不充分的：
只要 `camera_third_person` 仍在 `camera_names` 中，它的 RGB 仍会被渲染。

关闭 `camera_orbbec` pointcloud 不影响当前 perception，因为 pipeline 只消费
RGB 和 depth image。它还能移除 1 Hz pointcloud 投影/序列化的额外负载。

### 第一阶段验证

1. 配置测试确认只选择 `camera_orbbec`，且其 RGB/depth publisher 存在。
2. 重启仿真，不运行 pipeline、MoveIt goal、`grasp_demo`、机械臂或夹爪。
3. 确认：
   - `/camera_orbbec/color/image_raw` 存在；
   - `/camera_orbbec/depth/image_raw` 存在；
   - 不再发布 `/camera_third_person/*`；
   - RGB/depth 仍为 `1280x720`。
4. 在同一运行中至少测量 30 秒：
   - `/scene_clearance_bounds`；
   - `/scene_description`；
   - `/joint_states`。
5. 三者都必须达到 `>=18 Hz`。同时复核 AABB topic 仍为 6 个完整对象、
   QoS 仍为 reliable/volatile、日志中没有 bounds 构建或发布错误。

在当前单线程 `_sim_loop` 结构下，只要三个 topic 达到 `>=18 Hz`，就同时
证明 physics/state loop 已恢复到相同量级；此阶段不存在高频线程重复发布
低频旧快照的问题。

## 3. 第二阶段：按需降低主相机分辨率

只有第一阶段任一状态 topic 未达到 `18 Hz` 才进入第二阶段。

唯一新增配置变化：

```yaml
camera_size: [640, 480]
```

继续只渲染 `camera_orbbec`，其余配置、代码与验证方法不变。重新启动仿真后
再次测量至少 30 秒，不能用第一阶段与第二阶段样本混合计算平均值。

若第二阶段达到 `>=18 Hz`，必须在进入 Apple A3 前重新执行 3 次独立的
reset + fresh perception A1 只读验证。每次都重新捕获 640x480 RGB-D，检查：

- 自动 mask 选择仍正确；
- FoundationPose geometry center 与 scene truth 的三维误差仍小于 `10 mm`；
- 不使用 `FOUNDATIONPOSE_MASK_INDEX` 手工指定；
- 不执行 MoveIt、机械臂或夹爪命令。

A2 是与相机分辨率无关的纯抓取库筛选，不需要重复执行。

## 4. 第三阶段触发条件：不可变状态快照

只有“单路 `camera_orbbec` + 640x480”仍未达到 `18 Hz` 才允许进入第三阶段。
第三阶段属于新的线程/数据所有权设计，必须另行完成代码级设计审核，不在
本次最小配置实验中直接实现。

后续设计必须满足以下不可妥协条件：

1. 只有仿真所有者线程访问 MuJoCo `mjData`。
2. 每次完整 `mj_step()` 后复制同一时刻的 `xpos/xmat` 等所需状态。
3. writer 构造全新的不可变快照，再通过单次引用替换发布。
4. publisher 只读本地持有的快照引用，不原地修改或复用其 NumPy 数组。
5. `>=18 Hz` 必须是每秒至少 18 个不同序号/时间戳的新快照；禁止以 20 Hz
   重复发布同一个约 1.48 Hz 的旧快照来通过 `ros2 topic hz`。
6. 若 camera render 继续阻塞 `mj_step()`，只拆 publisher 无效；届时必须
   一并解决仿真状态更新与渲染节奏的所有权和调度。

## 5. 阶段门与停止条件

```text
代码事实核查
  -> 单路 camera_orbbec / 1280x720
      -> >=18 Hz：3 次 A1 只读验证 -> Apple A3 plan-only
      -> <18 Hz：单路 camera_orbbec / 640x480
          -> >=18 Hz：3 次 A1 只读验证 -> Apple A3 plan-only
          -> <18 Hz：停止；另行设计不可变快照/渲染调度
```

任何阶段出现以下情况都立即停止，不继续降级或进入 A3：

- `camera_orbbec` RGB/depth 缺失或尺寸不符合阶段配置；
- bounds 不完整、QoS 改变、几何错误或发布异常；
- 状态 topic 未达到门槛；
- 低分辨率 A1 任一次失败；
- 需要扩大到线程模型修改但尚未完成独立设计审核。

## 6. 明确禁止项

- 不把 Apple 写成专用分支。
- 不放宽 round-top fail-closed 规则。
- 不恢复 `0.1/0.05 m` 假边界 fallback。
- 不让新线程直接读取 live `mjData`。
- 不重复发布旧快照伪造 18 Hz。
- 不在本阶段运行 A3、MoveIt goal、`grasp_demo`、机械臂或夹爪。
