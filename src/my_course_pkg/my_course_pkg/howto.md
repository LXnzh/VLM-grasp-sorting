# 持续试验会话

## 启动

在一个终端中完成 ROS 环境初始化。API key 可以提前导出；如果未导出，
launch 会在启动任何子进程前隐藏输入一次。

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export VLM_API_KEY='替换为新 key'
ros2 launch ifl_air_ur_launch experiment_session.launch.py
```

不要把真实 key 写回此文件、launch 参数、命令历史或日志。此前曾保存为
明文的 key 必须在服务端撤销并轮换；从当前文件删除并不能清除 Git 历史中
已经存在的凭据。

## 启动交互

程序会直接显示 YAML 中配置的 18 个物品。输入最多 6 个逗号分隔的名称：

```text
banana, apple, racquetball
```

名称不区分大小写，逗号两边的空格可省略。未知、重复、空 token 或超过
6 个名称会要求重新输入。直接按 Enter 表示随机选择 6 个物品。

场景选择使用 YAML 的规范名 `racquetball`；进入每次试验的 instruction 后，
可以输入 `pick up the blue racquetball` 使用已固定的视觉别名。

物品只在会话启动时选择一次。后续 `/reset_sim` 会把同一批物品恢复到该
会话的初始位置，不会重新选择或重新随机物品。

## 每次试验

程序自动执行以下安全边界：

1. 停止自己拥有的 `moveit2_iface` 进程组；
2. 确认 `pgrep -a -x moveit2_iface` 无输出；
3. 启动唯一的新接口并检查 ROS 节点、action server 和 publisher 无重复；
4. 调用 `/reset_sim` 并等待稳定；
5. 提示输入新的 `Instruction:`；
6. 运行 `pipeline`；
7. 仅在 pipeline 成功后自动运行 `grasp_demo`。

成功后：

- 按 Enter 开始下一次试验；
- 输入 `q` 关闭接口、MuJoCo 和整个 launch。

等待输入期间 MuJoCo 保持开启。集成会话不会把 MoveIt、MuJoCo、RViz、夹爪、
robot-state 或 `moveit2_iface` 的持续运行日志回显到交互终端，因此
`Instruction:` 和 Enter/`q` 输入不会被刷屏覆盖。ROS launch 日志目录会在启动
时显示；接口日志仍保存在当前 trial 的 `moveit2_iface.log`。

## 失败处理

任意步骤失败都会停在当前步骤，不会自动继续到 perception、grasp 或运动。
终端会显示失败阶段和绝对日志路径；日志默认保存在：

```text
/tmp/my_course_experiment_sessions/<session>/trial_<n>/
```

检查并修复问题后，按 Enter 会从完整安全边界重新开始：重启接口、确认无
残留、重置仿真、重新输入 instruction，再运行新的 pipeline。输入 `q` 会
关闭整个会话。检测到不受本会话管理的残留 `moveit2_iface` 时，程序只报告
并停住，不会擅自杀掉该进程。

## 实时日志

保持集成会话终端用于交互，在第二个终端查看日志。启动时复制终端显示的 ROS
日志目录，然后运行：

```bash
tail -F /home/ros2/.ros/log/<本次-launch-目录>/launch.log
```

查看当前 trial 的接口或阶段日志：

```bash
SESSION_DIR=$(ls -1dt /tmp/my_course_experiment_sessions/* | head -n1)
TRIAL_DIR=$(ls -1dt "$SESSION_DIR"/trial_* | head -n1)
echo "$TRIAL_DIR"
tail -F "$TRIAL_DIR/moveit2_iface.log"
# pipeline 开始后可另开终端：
tail -F "$TRIAL_DIR/pipeline.log"
# grasp_demo 开始后可另开终端：
tail -F "$TRIAL_DIR/grasp_demo.log"
```

`tail -F` 会等待尚未创建的阶段日志，并在文件出现后继续跟踪。按 `Ctrl+C`
只会退出当前的日志查看，不影响主会话。

## 旧 launch 兼容性

原命令仍可使用，并且默认继续拥有一个内置的 `moveit2_iface`：

```bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py
```

不要在该旧完整 launch 旁边再启动独立的
`arm_api2 moveit2_iface.launch.py`。新 `experiment_session.launch.py` 会显式
关闭旧 launch 的内置接口，并由会话 supervisor 独占接口生命周期。
