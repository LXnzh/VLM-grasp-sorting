# Blue Racquetball 固定别名设计

**日期：** 2026-07-15

**状态：** 已批准

**范围：** `my_course_pkg` 的用户指令解析、GroundingDINO/SAM2 提示和
FoundationPose mask 匹配

## 背景

在包含 racquetball 和 tennis ball 的 `random` 场景中，用户输入
`pick up the racquetball`。VLM 正确选择了标准对象名 `racquetball`，但后续
GroundingDINO/SAM2 只收到生僻提示 `racquetball.`。检测器没有召回画面中的
蓝色 racquetball，反而把 hammer、tennis ball 和 banana 标成 racquetball。
mask 验证器随后选择 tennis ball，FoundationPose 又把 racquetball CAD 应用
到该错误 mask。最终 exact-bounds 安全门在任何机械臂运动前正确阻止了抓取。

用户希望使用更自然且更容易视觉定位的固定别名 `blue racquetball`。该别名
必须继续选择标准 YCB 对象 `racquetball`，但视觉检测必须使用 `blue ball`。

## 目标

- 接受包含短语 `blue racquetball` 的抓取指令。
- 将该短语确定性映射到标准对象身份 `racquetball`。
- GroundingDINO/SAM2 使用 `blue ball.`，不再使用生僻词作为该别名路径的
  视觉提示。
- FoundationPose、CAD 路径、抓取库、抓取类别和场景边界匹配继续使用
  `racquetball`。
- FoundationPose 接受 SAM2 返回的 `blue ball` 类名。
- 别名路径无法可靠找到 blue ball 时在感知阶段失败关闭，不产生姿态结果，
  更不启动机械臂。
- 保持普通 `racquetball` 输入和其余 17 个 YCB 对象的现有行为。

## 非目标

- 不建立任意颜色和任意物体的通用自然语言属性解析器。
- 不改变 FoundationPose 服务、racquetball CAD 或抓取候选生成算法。
- 不降低或关闭 approach-clearance、scene-bounds 或执行反馈安全门。
- 不通过禁止 racquetball 与 tennis ball 同场来掩盖感知错误。
- 不为普通 `racquetball` 输入自动改写视觉提示；本次只增加用户批准的固定
  别名路径。

## 方案选择

### 采用：身份与视觉提示分离的声明式固定别名

固定别名记录同时保存：

```text
instruction alias:           blue racquetball
canonical object identity:   racquetball
grounding prompt:            blue ball
accepted SAM2 class names:   blue ball
visual verification hint:    small smooth blue ball; not a yellow/green tennis ball
```

该方案不会把 `blue racquetball` 简单替换为 `racquetball` 后再次丢失颜色信息，
也不要求 VLM 每次动态生成可能变化的视觉提示。

### 未采用：仅增加名称同义词

如果只把用户短语映射成 `racquetball`，现有 `build_sam2_text_prompt()` 仍会生成
`racquetball.`，因此不能解决本次候选召回失败。

### 未采用：由 VLM 动态生成 grounding prompt

该方案可扩展到更多外观描述，但会引入新的非确定性输出和验证契约。当前只有
一个明确、稳定的固定别名，不需要扩大范围。

## 架构与职责

### 1. 固定别名解析

增加一个小型、无 I/O 的别名解析单元，集中保存别名元数据。解析规则为：

- 大小写不敏感；
- 允许 `blue` 与 `racquetball` 之间有一个或多个空白字符；
- 必须匹配完整单词，不接受其他单词中的子字符串；
- 完整用户指令中出现该短语即可匹配，例如
  `pick up the blue racquetball`；
- 显式调试变量 `VLM_CANDIDATE_OVERRIDE` 保持最高优先级。设置该变量时继续走
  现有 override 行为，不应用用户指令别名。

解析结果是结构化目标请求，而不是单个字符串。普通指令仍走现有 VLM 选择
逻辑；只有命中固定别名时才确定性返回 canonical identity 和 grounding prompt。

### 2. LLM/SAM2 数据流

命中别名后的数据流为：

```text
user instruction
  -> fixed alias resolver
  -> selected_object_name = racquetball
  -> grounding_prompt = blue ball
  -> SAM2 text_prompt = "blue ball."
```

`selected_object.json` 保留现有字段并增加可审计字段：

```json
{
  "user_instruction": "pick up the blue racquetball",
  "candidates": ["racquetball"],
  "selected_object_name": "racquetball",
  "matched_instruction_alias": "blue racquetball",
  "grounding_prompt": "blue ball",
  "sam2_text_prompt": "blue ball.",
  "accepted_sam2_class_names": ["blue ball"],
  "visual_target_description": "small smooth blue ball; not a yellow/green tennis ball"
}
```

普通指令继续产生当前 JSON，不写入上述五个别名专用字段。消费者在这些字段
缺失时必须使用当前标准对象名匹配行为，从而继续接受已有 JSON 和历史产物。

### 3. FoundationPose mask 匹配

FoundationPose 仍从 `selected_object_name` 解析标准目标和 mesh：

```text
selected_object_name = racquetball
mesh = 057_racquetball
```

mask 候选过滤另行读取 `accepted_sam2_class_names`。别名路径接受规范化后的
`blue ball`，同时不改变现有 `SAM2_CLASS_ALIASES` 兼容行为。类名比较沿用小写、
下划线转空格和首尾空白清理。

mask 验证器显示标准身份和别名视觉描述。若出现多个 `blue ball` 候选，验证器
必须依据“small smooth blue ball”选择；不得选择黄色或绿色 tennis ball。无法
给出高置信度唯一选择时必须进入下述失败路径。

### 4. 失败行为

别名路径必须失败关闭：

- SAM2 未返回 `blue ball`：抛出明确的 target-mask 错误，不调用外部
  FoundationPose 姿态估计服务。
- 多个候选且验证器没有返回合法、高置信度唯一选择：报错，不使用最高检测分
  fallback。
- 验证器选择不存在的索引或服务失败：报错，不使用 fallback。
- 成功前不得写入新的 `pose_result.json`；开始运行时继续清除旧姿态结果，避免
  下游使用陈旧数据。

普通对象继续使用当前候选验证和 fallback 策略；严格失败行为只属于本次固定
别名路径，避免无意改变已验证对象。

即使未来感知仍发生错误，现有 exact-bounds 和 approach-clearance 安全门保持
不变，继续作为独立的最后防线。

## 兼容性

- `pick up the racquetball`：保持当前标准名称路径。
- `pick up the blue racquetball`：走新固定别名路径。
- `BLUE   RACQUETBALL`：经规范化后走新路径。
- `VLM_CANDIDATE_OVERRIDE=racquetball`：保持现有 override 路径和提示语义。
- 其余 YCB 对象、抓取类别映射和所有环境变量契约不变。

## 测试设计

### 单元测试

- 固定别名解析接受大小写变化、多个空格及完整抓取句子。
- 固定别名解析拒绝部分单词和无关指令。
- `VLM_CANDIDATE_OVERRIDE` 优先于别名。
- 命中别名时输出 canonical identity `racquetball` 和 grounding prompt
  `blue ball`。
- SAM2 请求的 `text_prompt` 精确为 `blue ball.`。
- `selected_object.json` 同时保留用户指令、别名、标准身份、grounding prompt
  和 accepted class。
- FoundationPose 使用 `blue ball` annotation 作为 racquetball mask 候选。
- FoundationPose 仍解析 racquetball mesh，不能把 `blue ball` 当作 YCB mesh
  名称。
- 别名路径无候选、低置信度、多候选不确定和 verifier 异常均失败关闭。
- 普通 racquetball、tomato can 以及其他现有 mask-matching 测试不变并通过。

### 保存帧非运动验证

使用本次失败运行保存的 RGB 帧验证：

1. 输入 `pick up the blue racquetball`。
2. 日志显示 canonical identity 为 `racquetball`、SAM2 prompt 为
   `blue ball.`。
3. 蓝色球必须出现在 mask 候选中。
4. 黄色 tennis ball 不得成为最终 mask。
5. FoundationPose 请求使用 racquetball mesh。
6. 只验证感知产物，不执行 `grasp_demo`，不产生机械臂运动。

### 后续 live 门

保存帧验证通过后，重启完整仿真并检查 ROS graph 唯一性，运行一次 fresh
pipeline。只有 mask overlay、标准身份和姿态位置都对应蓝色 racquetball，才允许
进入独立的 plan-only 验证；本设计本身不授权机械臂运动。

## 完成标准

- 新旧单元测试全部通过。
- 保存帧上 blue racquetball 被召回且 tennis ball 未被选中。
- 输出 JSON 能清楚区分用户别名、视觉提示和标准对象身份。
- FoundationPose 使用 racquetball CAD，并在感知不确定时失败关闭。
- 无安全阈值、场景布局或其他对象行为回归。
