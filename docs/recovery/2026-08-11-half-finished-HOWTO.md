# grasp_stable 使用指南

基于 ROS 2 Humble + MuJoCo 仿真的 UR10e 机器人抓取工作区。

## 工作区结构

```
grasp_stable/
├── .devcontainer/              Docker Dev Container 配置
├── gui_manager.py              一键启动 GUI（旧版，需 terminator）
├── setup_ros.bash              source ROS2 + workspace
├── src/
│   ├── my_course_pkg/          ★ 抓取核心包
│   │   └── my_course_pkg/
│   │       ├── grasp/          抓取模块
│   │       │   ├── demo.py             主入口
│   │       │   ├── executor.py         运动执行器
│   │       │   ├── grasp_selector.py   抓取姿态选择
│   │       │   ├── candidate_ranker.py 候选排序
│   │       │   ├── trajectory_planner.py 轨迹规划
│   │       │   ├── pick_place_planner.py pick-place（含 safe_place 模式）
│   │       │   ├── gripper_control.py  夹爪控制
│   │       │   ├── config.py           所有配置参数
│   │       │   └── transforms.py       坐标变换
│   │       ├── perception/     感知模块
│   │       │   ├── llm_sam2.py         LLM + SAM2 分割
│   │       │   └── foundationpose.py   6D 位姿估计
│   │       └── pipeline.py     感知 pipeline 主流程
│   ├── ifl_air_mujoco_sim/     MuJoCo 仿真后端
│   ├── ifl_air_ur_launch/      launch 文件
│   ├── arm_api2/               MoveIt 封装
│   ├── arm_api2_py/            Python 客户端
│   └── sim_pick_place/         示例 pick-and-place
```

## 快速开始

### 1. 打开 Dev Container

VS Code → Dev Containers → Reopen in Container

> 如有 NVIDIA GPU，将 `.devcontainer/Dockerfile` 第一行改为：
> `FROM nvidia/cuda:12.8.1-devel-ubuntu22.04`

### 2. 构建

```bash
cd /home/ws
colcon build --symlink-install
source install/setup.bash
```

### 3. 准备 MuJoCo 虚拟环境

```bash
cd src/ifl_air_mujoco_sim
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 运行

### 方式一：分终端启动

**终端 1** — MuJoCo 仿真：

```bash
cd /home/ws/src/ifl_air_mujoco_sim
source .venv/bin/activate
python3 ros2_main.py sim.headless=true
```

**终端 2** — ROS2 机器人 + MoveIt：

```bash
source /home/ws/setup_ros.bash
ros2 launch ifl_air_ur_launch cell_small_ur_orbbec_robotiq_mujoco.launch.py launch_rviz:=false
```

**终端 3** — MoveIt 规划：

```bash
source /home/ws/setup_ros.bash
ros2 launch ifl_air_ur_launch moveit_cell_small_ur_orbbec_robotiq.launch.py launch_rviz:=false
```

**终端 4** — 抓取演示：

```bash
source /home/ws/setup_ros.bash
ros2 run my_course_pkg grasp_demo
```

### 方式二：一条命令启动仿真 + MoveIt

```bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py
```

然后在另一个终端运行 `ros2 run my_course_pkg grasp_demo`。

## 可用的 ROS2 命令

| 命令 | 功能 |
|------|------|
| `ros2 run my_course_pkg verify_init_pose` | 检查机器人初始位姿 |
| `ros2 run my_course_pkg rgbd_saver` | 保存一帧 RGB-D 图像 |
| `ros2 run my_course_pkg pipeline` | 运行感知 pipeline（LLM→SAM2→FoundationPose） |
| `ros2 run my_course_pkg grasp_demo` | 执行抓取演示 |
| `ros2 run my_course_pkg grasp_eval` | 抓取评估 |

## 感知 Pipeline

`ros2 run my_course_pkg pipeline` 按四步执行：

1. **验证初始位姿** — 确保机器人在正确起始姿态
2. **采集 RGB-D** — 拍一帧彩色+深度图
3. **LLM + SAM2 目标选择** — 自然语言指令选择目标物体
4. **FoundationPose 位姿估计** — 估计目标 6D 位姿

需要设置 `VLM_API_KEY` 环境变量。

## 关键环境变量

### 必须设置

| 变量 | 说明 |
|------|------|
| `VLM_API_KEY` | VLM/LLM API 密钥（pipeline 使用） |

### 抓取参数（可选，有默认值）

| 变量 | 说明 | 默认值 |
|------|------|--------|
| `GRASP_ROOT` | 抓取姿态库路径 | `/home/ws/grasps` |
| `GRASP_EXECUTION_MODE` | 执行模式：`lift_return` 或 `safe_place` | `lift_return` |
| `GRASP_APPROACH_DIST` | 接近距离 (m) | `0.1` |
| `GRASP_LIFT_HEIGHT` | 提升高度 (m) | `0.2` |
| `GRASP_Z_OFFSET` | 抓取 Z 偏移 (m) | `-0.02` |
| `SIDE_GRASP_Z_OFFSET` | 侧抓 Z 偏移 (m) | `0.0` |
| `DROP_X` / `DROP_Y` / `DROP_Z` | 放置位置 | `-0.55, -0.45, 0.35` |
| `GRASP_GRIPPER_EFFORT` | 夹爪力度 | `140.0` |
| `GRASP_CANONICALIZE_TABLETOP_OBJECT_POSE` | 标准化桌面物体位姿 | `true` |

### 调试参数

| 变量 | 说明 |
|------|------|
| `GRASP_DEBUG_STOP_AT_GRASP` | 到达抓取位后暂停 |
| `GRASP_DEBUG_STOP_AFTER_CLOSE` | 夹爪关闭后暂停 |
| `GRASP_DEBUG_STOP_AFTER_LIFT` | 提升后暂停 |
| `GRASP_DEBUG_STOP_BEFORE_RELEASE` | 释放前暂停 |
| `VLM_CANDIDATE_OVERRIDE` | 强制指定目标物体名（跳过 VLM） |

### 推荐的环境变量组合

```bash
export GRASP_ROOT=/home/ws/src/my_course_pkg/grasps
export GRASP_CANONICALIZE_TABLETOP_OBJECT_POSE=1
export SIDE_GRASP_Z_OFFSET=-0.045
export DROP_Z=0.45
```

## 与 feature/gui-pipeline 的区别

`grasp_stable` 相比 `feature/gui-pipeline`：

- **有** `safe_place` 放置模式（场景感知放置、碰撞网格避让）
- **有** 夹爪打开失败重试机制
- **有** 侧抓碰撞检测（corridor clearance）
- **没有** round_top 抓取策略（球形水果顶部抓取）
- **没有** 锤子重心平衡抓取
- **没有** GUI 控制台（gui_manager.py 是旧版，依赖 terminator）
- **没有** PBVS 动态跟随抓取

完整配置参数见 `src/my_course_pkg/my_course_pkg/grasp/config.py`。

## AI 交接速查（避免重复排错）

这个章节用于给后续会话快速对齐现场状态。每次开始前先看这一段，优先走“最短可运行路径”。

### 最近一次已确认问题（2026-08-11）

- 现象：`ros2 run my_course_pkg pipeline` 报 `ModuleNotFoundError: No module named 'arm_api2_py'`
- 原因：只编译了 `my_course_pkg`，没有确保 `arm_api2_py` 被构建并进入 overlay 环境
- 结论：`pipeline` 运行依赖 `arm_api2_py`（Python 包），不要求 `arm_api2`（C++ 包）必须编译成功

### 最近一次构建状态（2026-08-11）

- `arm_api2` 构建失败（大量 `yaml-cpp` 链接错误，`undefined reference to YAML::...`）
- 该失败属于 `arm_api2` C++ 链接配置问题，不等于 `my_course_pkg pipeline` 必然不可用
- `my_course_pkg` 已可单独构建成功

### 最短启动路径（优先执行）

在 `/home/ws` 下执行：

```bash
colcon build --packages-select arm_api2_py my_course_pkg
source install/setup.bash
ros2 run my_course_pkg pipeline
```

如果仍提示包不可见，先确认你在同一个终端里 `source install/setup.bash` 之后再运行 `ros2 run`。

### 每次会话开局检查清单

1. 当前目录是否为 `/home/ws`
2. 当前终端是否已执行 `source install/setup.bash`
3. `arm_api2_py` 是否已编译（必要时重跑上面的最短启动命令）
4. 再运行 `ros2 run my_course_pkg pipeline`，观察是否出现 `Instruction:` 交互输入提示

### 后续待修（工程层面）

- `arm_api2` 的 `yaml-cpp` 链接问题需要在其 `CMakeLists.txt` 中补齐正确链接配置
- 该修复与“让 pipeline 先跑起来”是两件事，建议分开处理，避免阻塞日常验证
