# 仿真自瞄桥接器

`auto_aim_bridge` 将 `rm_aim_sim` 接到 `../auto-aim-new-0` 的实际 Detector、PnP、
Tracker 和 MPC Planner。输入数据与状态估计器可以在运行时分别选择。

两条输入链路为：

| 输入模式 | 数据源 | 用途 |
| --- | --- | --- |
| `exact-corners`（默认） | `state.camera.armors` 的精确 TL/TR/BR/BL | 单独验证 PnP/UV、跟踪、规划和控制 |
| `raw-frame` | 同一请求的 `frame-*.raw` 与 `frame.json` | 验证渲染图像、YOLO、所选估计器和控制全链路 |

两种状态估计器为：

| 估计器 | 启动参数 | 主要用途 |
| --- | --- | --- |
| PnP（默认） | `--estimator=pnp`，也可省略 | 每帧由 Solver/PnP 产生三维装甲板测量，保持原有基线 |
| 单装甲板 UV-EKF | `--estimator=uv-ekf` | PnP 初始化 UV 分段，后续 UV 状态直接用 TL/TR/BR/BL 像素更新；Tracker 仍以 PnP 完成注入前关联 |

`exact-corners` 与 `raw-frame` 都支持这两种估计器。输入链路只负责生成
`std::list<auto_aim::Armor>`；估计器完成测量后，仍共用原有 Target、Tracker 状态机、
Planner、控制新鲜度和开火安全门。`raw-frame` 使用上游同步
`auto_aim::YOLO(config, false)`；YOLOv5 可继续按 `use_traditional` 对候选角点做传统灯条
二次修正。

桥接器默认以 dry-run 方式运行，只计算和记录结果。只有显式传入 `--control` 才会向
仿真发送云台指令，单独使用 `--control` 仍保持 `fire=false`。必须再显式传入
`--allow-fire`，才会在安全门限通过时转发 Planner 的开火许可。

相关协议和坐标约定见：

- [通用接入指南](../docs/integration.md)
- [C++ 接入说明](../docs/cpp_integration.md)
- [共享内存协议](../docs/protocol.md)

## 数据链路

```mermaid
flowchart LR
    S[state.json] --> I{--input}
    I -->|exact-corners| C[camera.armors<br/>TL/TR/BR/BL]
    C --> A[构造 auto_aim::Armor]
    I -->|raw-frame| R[request.json]
    R --> F[frame-请求ID.raw<br/>同帧 frame.json]
    F --> B[RGB/RGBA 转自有 BGR]
    B --> Y[auto_aim::YOLO]
    Y --> A
    S --> Q[生命周期与最新控制时钟]
    F --> K[同帧 K、D、R、t、姿态、sim_time]
    A --> E{--estimator}
    K --> E
    E -->|pnp| P[每帧 Solver / PnP]
    E -->|uv-ekf 未初始化| PS[PnP 初始化种子]
    E -->|uv-ekf 已初始化| U[SimpleUvEkf<br/>像素关联与更新]
    PS --> U
    P --> T[原有 Tracker / Target EKF]
    U -->|初始化或更新接受| T
    U -->|无可用更新| TP[Tracker / Target<br/>prediction-only]
    T --> L[原有 Planner / MPC 与弹道]
    TP --> L
    Q --> L
    L --> D{运行模式}
    D -->|dry-run| O[只写诊断]
    D -->|--control| M[command.json<br/>绝对角度与前馈]
    M --> FD{--allow-fire 且<br/>Planner 允许开火?}
    FD -->|是| X[持续开火许可]
    FD -->|否| N[fire=false]
    S -. truth 仅在 exact-corners 完成后评分 .-> O
    F -. camera.armors 只作 raw 诊断基准 .-> O
```

### 精确角点模式

每个 `camera.armors[].corners` 必须按以下语义顺序排列：

```text
TL, TR, BR, BL
```

像素坐标以图像左上角为原点，x 向右，y 向下。适配器原样保留这四个点，并根据
`appearance.robot` 和 `appearance.team` 构造自瞄仓库中的 `Armor`：

| 仿真车型 | 自瞄编号 | PnP 尺寸 |
| --- | --- | --- |
| `hero` | 英雄 1 | 大装甲 230 × 56 mm |
| `infantry_3` | 步兵 3 | 小装甲 135 × 56 mm |
| `infantry_4` | 步兵 4 | 小装甲 135 × 56 mm |
| `infantry_5` | 步兵 5 | 小装甲 135 × 56 mm |
| `sentry` | 哨兵 | 小装甲 135 × 56 mm |

`camera.armors[].id` 是当前仿真中四个物理板面的编号，范围为 0 到 3。桥接器保留它
用于同一快照内的诊断和真值配对，Tracker 仍使用自己的装甲槽位关联逻辑。

### 真实图像模式

`raw-frame` 主线程一次只保留一个图像请求。它原子写入 `request.json`，等待匹配当前
`request_id`、publisher、session 和静态上下文的 `frame.json`，再只打开准确的
`frame-<request_id>.raw`。客户端检查普通文件、编码、分辨率、文件名和精确字节数，完整
复制 RGB/RGBA 并转换成拥有独立存储的 BGR 后，才允许发送下一请求。

图像和算法必须使用该帧 `frame.json` 自带的 K、畸变、外参、云台四元数、武器参数、
tick 和 `sim_time`。最新 `state.json` 只负责判断发布者生命周期、控制开关、状态新鲜度和
Planner 要外推到的当前仿真时间，绝不会覆盖旧图像的同帧姿态。

`frame.camera.armors` 仍随元数据存在，但 raw 适配器不解析它，也不使用
`appearance.robot` 生成类别；只有 YOLO 像素输出能进入所选估计器。日志可以保留这些
理想角点，用于检测完成后的离线角点误差评分。

PnP 模式中，`Tracker::track()` 调用实际 `Solver` 完成 IPPE、可选精简自由度优化以及
后续关联，然后使用原有 EKF 更新目标。UV-EKF 模式只在初始化或重置后的新分段使用 PnP
种子，具体生命周期见下一节。桥接器不会用 `state.truth` 的三维位置覆盖任何估计结果。

Planner 使用当前输入快照的 `weapon.speed`、`weapon.delay`、显式算法时间和 Tracker
输出生成 yaw、pitch、角速度、角加速度及开火意图。每次新观测先由所选估计器更新
Tracker/Target，算法线程随后按最新已知 `sim_time` 合并执行最高约 100 Hz 的异步重规划。
有效控制结果以约 50 Hz 写回仿真的最新值邮箱。

## 状态估计器切换

### PnP 基线

`--estimator=pnp` 是默认模式。每帧候选角点进入 Solver/PnP，得到世界系装甲板位置和
姿态，再由原有 Tracker/Target EKF 完成整车状态估计并交给 Planner。这条路径用于与现有
算法和历史结果直接对照；省略 `--estimator` 与显式写出 `--estimator=pnp` 等价。

### 单装甲板 UV-EKF

`--estimator=uv-ekf` 启用 `auto_aim::SimpleUvEkf`。它维护一个单装甲板状态：

```text
[X, Y, Z, VX, VY, VZ, armor_yaw, armor_yaw_rate]
```

每个 UV 分段开始时，桥接器用同一候选的 PnP 位置和姿态初始化位置、yaw 以及固定的
pitch/roll；初始速度和角速度为零。初始化以后，当前帧 PnP 不再进入 UV 状态，滤波器只用：

- 原始 TL/TR/BR/BL 像素；
- 当前帧的相机标定与云台到世界旋转；
- 严格递增的仿真算法时间；
- 颜色、类别、装甲尺寸和像素创新 NIS 做候选关联。

这里“PnP 只提供 seed”指 UV 滤波状态和最终注入 Target 的板测量。为保留现有 Tracker 的
候选/板槽关联与诊断边界，Tracker 内部仍可能对 UV 选中的单个候选执行 Solver；该帧 PnP
的 XYZ/yaw 会保存在 `tracker_measurements[].pnp_*` 供对照，但随后由 UV 后验覆盖，不作为
`used_*` 测量更新 Target。

当前 measurement override 位于 Tracker 的 Solver 和板槽关联之后，因此当帧 PnP 成功仍是
UV 后验能够注入的前置门槛，板槽编号也仍由 PnP 关联决定。PnP 求解失败时，即使 UV 像素
更新已经接受，该帧也不会形成 Tracker 测量；应结合 `measurement_ready`、
`measurement_injected` 和 `measurement_found` 区分这些阶段。若要彻底移除该依赖，需要在
上游 Tracker 增加 PnP 前的测量/关联接口，或实现完整整车 UV 观测模型。

只有唯一候选通过像素关联门限，并且 UV 更新被接受时，桥接器才把该单板的世界系
位置和 yaw 注入原有 Tracker。无检测、关联歧义、重投影/NIS 拒绝时，本帧不会偷偷改用
PnP 测量，而是让现有 Target 执行 prediction-only。连续拒绝、测量超时、非法时间间隔或
数值异常会结束当前 UV 分段；后续候选需要重新经过一次 PnP seed 才能建立新分段。

UV-EKF 没有替换整车模型。Tracker 的确认、短时丢失和生命周期状态机仍然生效，原有
Target 继续维护旋转中心、板槽、半径与高度，Planner 仍只接收原 Target。这样两种估计器
共用相同的弹道、选板、MPC、控制新鲜度和开火安全门，不会把单板 8 维状态误当成整车
12 维状态。

两种输入都能使用 UV-EKF：

- `exact-corners + uv-ekf` 用于检查 UV 投影模型、时间和后端控制，不包含神经网络误差；
- `raw-frame + uv-ekf` 使用 YOLO 输出的实际像素四角，覆盖渲染、检测、UV 更新和控制链。

估计器在进程启动时选择，运行中不会热切换。需要比较 PnP 与 UV-EKF 时，应分别启动两个
新的运行并使用不同输出目录，避免 Tracker、Planner warm state 和日志相互混合。

## 坐标、姿态与标定

仿真相机光学系为 x 向右、y 向下、z 向前；云台系和 Solver 使用 x 向前、y 向左、
z 向上。每个状态快照提供当前 K、畸变和相机到云台的变换：

```text
p_gimbal = R_camera2gimbal * p_camera + t_camera2gimbal
```

`gimbal.quaternion_wxyz` 按 w、x、y、z 排列，并把云台系向量旋转到世界系。桥接器归一化
该四元数，再传给 `Solver::set_R_gimbal2world()`。它还检查 camera timestamp、camera tick
与外层 `sim_time`、tick 属于同一个快照。精确角点使用 state 快照；真实图像使用 frame
快照，二者都不会混用不同时刻的姿态和观测。

相机 K、畸变、R、t 都来自当前输入快照，不复制实车 YAML 中的标定。仿真不模拟额外
IMU 安装旋转，所以有效配置将 `R_gimbal2imubody` 设为单位阵。

## 真值边界

`exact-corners` 中的 `state.camera.armors` 是仿真根据精确几何投影得到的理想二维观测，
所以该模式绕过检测器和分类器，只验证二维角点之后的后端链路。

`raw-frame` 中 `frame.camera.armors` 不进入 Detector、PnP、Tracker 或 Planner。测试会把
该字段和 `appearance.robot` 故意替换成无效值，确认 raw 适配仍只输出 BGR 图像。真实图像
模式的 frame 元数据按协议不含 `truth`；理想角点只可在算法完成后作为检测评分基准。

`state.truth.armors` 只在 Solver、Tracker 和 Planner 都完成之后使用，用于写出：

- `position_error_m`：PnP 世界位置与对应物理装甲中心的误差；
- `yaw_error_rad`：PnP 内向法线 yaw 与仿真真值的包角误差。

删除或修改 `state.truth` 不应改变 Tracker 和 Planner 的输出。离线测试专门检查了这项
性质。`observations.jsonl` 会保存完整原始 state，因此日志中仍会包含 truth；这不表示
truth 进入了算法输入。

## 环境与依赖

桥接器使用 Linux 本机 `/dev/shm` 邮箱，需要 CMake 3.16、C++17，并直接编译
`auto-aim-new-0` 中的同步 YOLO、传统角点修正、PnP、Tracker、Planner 和 TinyMPC 源码。
它不编译工业相机、CAN、串口或硬件入口。

推荐的工作区结构为：

```text
rm_project/
├── rm_aim_sim/
│   └── auto_aim_bridge/
└── auto-aim-new-0/
```

Ubuntu / Debian 可以安装以下构建依赖：

```bash
sudo apt update

sudo apt install -y \
  build-essential \
  cmake \
  git \
  pkg-config \
  libopencv-dev \
  libeigen3-dev \
  libyaml-cpp-dev \
  nlohmann-json3-dev \
  libfmt-dev \
  libspdlog-dev \
  libg2o-dev
```

`raw-frame` 需要 OpenVINO Runtime。当前上游工程默认使用
`/opt/intel/openvino_2024.6.0/runtime/cmake`；CMake 在该位置存在时自动采用它，也可以显式
传入：

```bash
-DOpenVINO_DIR=/absolute/openvino/runtime/cmake
```

即使只打算运行 `exact-corners`，当前单一可执行文件也会编译 YOLO，因此构建时仍需要
OpenVINO。若 g2o 或其他依赖安装在非系统路径，可以补充
`-DCMAKE_PREFIX_PATH=/absolute/prefix`。

首次使用仿真时，在 `rm_aim_sim` 根目录准备 Godot 并导入资源：

```bash
python3 tools/setup_godot.py

./run.sh --headless --editor --import --quit
```

## 编译

以下命令均从 `rm_aim_sim` 根目录执行。`AUTOAIM_ROOT` 必须是绝对路径：

```bash
cmake \
  -S auto_aim_bridge \
  -B .tools/auto-aim-bridge \
  -DCMAKE_BUILD_TYPE=Release \
  -DAUTOAIM_ROOT="$(realpath ../auto-aim-new-0)" \
  -DBUILD_TESTING=ON

cmake \
  --build .tools/auto-aim-bridge \
  --parallel
```

默认关闭 Reduced-DoF PnP。若要比较约束平移与 yaw 优化，应使用独立构建目录：

```bash
cmake \
  -S auto_aim_bridge \
  -B .tools/auto-aim-bridge-reduced-pnp \
  -DCMAKE_BUILD_TYPE=Release \
  -DAUTOAIM_ROOT="$(realpath ../auto-aim-new-0)" \
  -DAUTOAIM_USE_REDUCED_DOF_PNP=ON \
  -DBUILD_TESTING=ON

cmake \
  --build .tools/auto-aim-bridge-reduced-pnp \
  --parallel
```

修改 `auto-aim-new-0` 中被桥接器直接编译的源码后，需要重新构建桥接器。

## 启动仿真

`exact-corners` 可以使用图形或无界面模式。无界面模式适合自动测试：

```bash
./run.sh --headless -- --remote
```

`raw-frame` 必须使用图形模式，因为 Godot 的 sensor viewport 在 `--headless` 下不发布
渲染帧：

```bash
./run.sh -- --remote
```

`--remote` 会启用外部控制。仿真和桥接器的 channel 必须一致；同一个 channel 同时只能
有一个客户端，运行桥接器时不要再启动 `control_demo.py`、`camera_demo.py` 或其他控制端。

当前 Tracker 将图像中心固定为 `(720, 540)`，桥接器会拒绝非 1440 × 1080 的相机。
可以改变视场角，实时 K 会被写入有效配置，但分辨率必须保持 1440 × 1080。

## 先运行 dry-run

精确角点 dry-run：

```bash
.tools/auto-aim-bridge/auto_aim_bridge \
  --input=exact-corners \
  --estimator=pnp \
  --dry-run \
  --duration=10 \
  --output=/tmp/auto_aim_bridge_dry_run
```

真实图像与 YOLO dry-run：

```bash
.tools/auto-aim-bridge/auto_aim_bridge \
  --input=raw-frame \
  --estimator=pnp \
  --dry-run \
  --duration=10 \
  --output=/tmp/auto_aim_bridge_raw
```

要检查 UV-EKF，可以把上述命令中的 `--estimator=pnp` 改为
`--estimator=uv-ekf`。切换估计器不改变输入源：`exact-corners` 仍来自精确投影，
`raw-frame` 仍来自 YOLO。

dry-run 是默认模式，因此可以省略 `--dry-run`。两种输入都会运行到 Planner 和诊断记录，
但不会创建或发布 `command.json`，云台不会因桥接器而运动。

`--output` 指向的目录必须不存在或为空。省略该参数时，程序会在当前工作目录下创建：

```text
logs/auto_aim_bridge_<Unix毫秒>_<进程号>/
```

终端会打印当前运行模式、输入、estimator、channel、输出目录和 generation，并每秒报告
一次仿真时间、Tracker 状态、控制许可及最近计划角度。它适合快速确认程序仍在工作，
定量分析应使用输出文件。

## 开启云台闭环

确认 dry-run 的估计器和规划输出正常后，退出前一个桥接器，再开启闭环。以下两条命令均
省略 `--duration`，会持续运行到按下 Ctrl+C；也省略 `--output`，每次自动创建新的日志
目录。

精确角点加 UV-EKF：

```bash
.tools/auto-aim-bridge/auto_aim_bridge \
  --input=exact-corners \
  --estimator=uv-ekf \
  --control
```

真实图像、YOLO 加 UV-EKF：

```bash
.tools/auto-aim-bridge/auto_aim_bridge \
  --input=raw-frame \
  --estimator=uv-ekf \
  --control
```

要持续运行原 PnP 基线，把 `--estimator=uv-ekf` 改成 `--estimator=pnp`，或直接省略该参数。
`raw-frame` 必须配合前台图形仿真，不能使用 `--headless`。

实际图像模式中检测或跟踪短暂失效时，安全门会发布 `control=false, fire=false`，新鲜检测
重新确认后才恢复控制。UV-EKF 更新拒绝时同样只做预测，不会因为控制已开启而回退到 PnP
测量。

控制输出使用协议定义的绝对弧度：yaw 向左为正，pitch 向下为正。桥接器从有效配置中
强制设置：

```yaml
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
gimbal_yaw_zero: 0.0
gimbal_pitch_sign: -1.0
```

因此 Planner 的结果可以直接映射到仿真协议。角速度和角加速度前馈也会一并发送。

只传入 `--control` 时仍固定发布 `fire=false`，可先验证云台闭环而不消耗目标血量。

## 允许 Planner 开火

上述持续运行命令默认都没有 `--allow-fire`，因此只控制云台而不会开火。先检查日志、云台
跟随、UV 更新接受率和 Tracker 状态；确认仅控制模式稳定后，再在原命令末尾显式增加
`--allow-fire`：

```bash
.tools/auto-aim-bridge/auto_aim_bridge \
  --input=exact-corners \
  --estimator=uv-ekf \
  --control \
  --allow-fire
```

`--allow-fire` 不能单独使用，必须与 `--control` 同时出现。省略 `--duration` 时持续运行到
按下 Ctrl+C；省略 `--output` 时自动创建新的日志目录。

实际发布 `fire=true` 必须同时满足：

- 用户显式给出 `--control --allow-fire`；
- 当前状态、观测、generation 和规划均通过控制新鲜度检查；
- Tracker 当前状态严格为 `tracking`，`detecting`、`temp_lost` 和 `lost` 均不开火；
- Planner 的 `plan.fire` 为 true；
- IPC 客户端发送时状态仍然新鲜，并且实际发布 `control=true`。

Planner 根据选板射击窗口与 `fire_thresh` 产生持续许可。仿真收到持续的 `fire=true` 后按
`weapon.rate` 周期开火，并在 `weapon.delay` 后出膛；它不是每个命令包只打一发。桥接器
会在首次构造 Planner 时把当前 `weapon.delay` 写入有效配置，使目标预测包含这段出膛
延迟。上游 Planner 在进程内复用，因此运行中修改 `weapon.delay` 会提示重启桥接器，
不会在旧延迟下继续开火。

`fire=false` 会停止新增触发，但已经进入仿真延迟队列的弹丸仍可能出膛；失去有效控制、
状态超时、切换会话或退出桥接器会发送 `control=false, fire=false` 并取消待出膛事件。
已经出膛的弹丸仍会完成碰撞结算。

默认步兵只有 350 HP，连续命中后会战亡，理想角点随之消失，Tracker 最终停止控制和
开火。这属于预期行为。长时间射击实验可以在仿真界面恢复血量，或在场景配置的
`[target]` 中提高 `max_health`。

## 命令行参数

```text
Usage: auto_aim_bridge [options]

  --channel=NAME      仿真 IPC channel，默认 default
  --input=MODE        exact-corners（默认）或 raw-frame
  --estimator=MODE    pnp（默认）或 uv-ekf
  --config=PATH       Detector/Tracker/Planner 基础 YAML，默认 config/default.yaml
  --output=PATH       新建或空的日志目录
  --duration=SECONDS  按墙钟运行指定秒数；省略时运行到 Ctrl+C
  --dry-run           只计算和记录，默认模式
  --control           启用云台控制，默认仍不开火
  --allow-fire        转发新鲜的 Planner 开火许可；要求 --control
  --help              显示帮助
```

`--dry-run` 与 `--control` 不能同时使用，`--allow-fire` 不能脱离 `--control` 使用。
`--duration` 必须是有限正数。channel 只能包含 1 到 80 个英文字母、数字、下划线或
连字符。非法输入模式或估计器名称会直接退出，不会自动退回理想角点或 PnP。

## 配置的来源与覆盖规则

[默认配置](config/default.yaml) 保存 YOLO、传统角点修正、Tracker、PnP、Planner、弹道和
MPC 参数。每次建立新 generation 时，桥接器复制基础 YAML，并用当前输入快照覆盖以下
字段：

- `camera_matrix`；
- `distort_coeffs`；
- `R_camera2gimbal`；
- `t_camera2gimbal`；
- `enemy_color`；
- `R_gimbal2imubody`、`gimbal_yaw_zero` 和 `gimbal_pitch_sign`；
- `high_speed_delay_time` 和 `low_speed_delay_time`，两者在首次构造 Planner 时取当前
  `weapon.delay`。

覆盖后的配置写入 `effective_config_<generation>.yaml`，Solver 和 Tracker 从该文件重建。
当前弹速不写死在 YAML 中，而是每次调用 Planner 时从对应观测快照传入。

`raw-frame` 另外读取以下 Detector 配置：

- `yolo_name`：`yolov5`、`yolov8` 或 `yolo11`；默认 `yolov5`；
- `device`、`min_confidence`、`use_roi` 和 `roi`；
- 当前 YOLO 对应的 `*_model_path`；
- `classify_model`；
- `use_traditional` 以及传统灯条检测阈值。

相对模型路径先相对传入 YAML 的目录查找，再相对 `AUTOAIM_ROOT` 查找；找到后写成绝对路径
保存到 `detector_config.yaml`。因此从任意工作目录启动都不会把 `assets/...` 错解到仿真
仓库。找不到模型或 OpenVINO 无法编译模型时，算法线程会明确报错退出，不会静默切换输入。
当前默认使用 `assets/yolov5.xml` 的装甲板输出格式；`assets/best2-sim.onnx` 是 buff 五关键点
模型，输出契约不同，不能直接替换装甲板 YOLO 路径。

`uv-ekf` 另外读取 `uv_ekf` 和 `uv_ekf_association` 配置组。前者包含像素噪声、线/角
加速度噪声、初始协方差、最大 dt、重投影门限和 NIS 概率；后者包含无测量超时、最大连续
拒绝帧数、候选关联 NIS 门限以及第一名与第二名的歧义门限。相机 K、畸变和手眼外参仍由
当前 generation 的输入快照覆盖，并写入同一份 `effective_config_<generation>.yaml`。

这些门限同时作用于 `exact-corners` 和 `raw-frame`，但两者的像素误差分布并不相同。
比较时应保留有效配置和 estimator 模式，不应只比较命中率而忽略 UV 更新接受率与重置次数。

基础配置中的 `Q_yaw`、`Q_pitch` 必须分别包含两个数，`R_yaw`、`R_pitch` 必须分别包含
一个数。桥接器在启动时检查这些形状和正值约束，避免上游 TinyMPC 读取越界。

配置文件在进程启动时读取一次。运行期间编辑 YAML 不会热更新；修改后应退出并重启。

## 时间、线程和生命周期

主线程独占 `LocalSimClient`，轮询最新 state、请求/复制 raw frame 并负责发送命令。算法
线程独占 YOLO、Solver、Tracker 和 Planner。两者之间分别保留一个待处理 Observation 和
一个重规划请求；较新的请求会覆盖尚未开始处理的旧请求，从而优先使用最新观测和规划
时刻，避免积累检测延迟。Detector 对象从不跨线程并发调用。

raw 邮箱同一时间只有一个请求在途。服务器发布新帧后会在下一请求确认时删除上一 raw，
所以主线程必须先完成像素复制和 BGR 转换。等待帧超过 5 秒会报错并提示使用图形模式；
控制新鲜度仍按 100/200 ms 门限更早撤销旧计划，不会等到 5 秒才停控。

新 Observation 根据 estimator 模式执行每帧 PnP，或执行 UV 初始化/预测/像素更新，然后
进入 Tracker/Target 和目标确认。只要状态、generation 和结果仍然有效，主线程就每
10 ms 请求一次以当前最新 `sim_time` 重规划。算法线程串行处理观测与规划，请求繁忙时会
合并，因此 100 Hz 是目标上限，不是硬实时保证。命令循环每 20 ms 检查一次最新计划，
目标发送上限约为 50 Hz。

Tracker 和 Planner 需要 `steady_clock` 类型时间。每个 generation 会把第一帧
`sim_time` 锚定到一个本地 steady 时间点，后续只使用仿真时间差推进算法。暂停、重置或
仿真时间回退后不会把 `sim_time` 直接和主机时钟相减。

`wall_unix_sec` 只用于首次连接或 `publisher_id` 改变时拒绝崩溃遗留的 state。对同一
publisher 的后续 publication，客户端使用本地单调接收时间判断新鲜度，避免系统墙钟
跳变影响正在运行的控制。

以下变化会建立新 generation：

- `publisher_id` 或 `session_id` 改变；
- 暂停状态或外部控制开关改变；
- 车型、阵营或外观配置改变；
- 分辨率、K、畸变、相机外参或云台原点改变；
- 相机亮度增益改变；
- 武器配置改变；
- `sim_time` 回退。

切换 generation 时，桥接器还取消待处理 frame 请求，清除待处理观测和旧控制结果，并
重建 Solver、Tracker 和所选估计器。UV-EKF 不会跨 generation 保留单板状态或 PnP seed。
Planner 的目标选择会通过空目标调用清除。由于上游 Planner 重复构造会泄漏其原始
求解器句柄，当前实现保留同一个 Planner，TinyMPC 的内部 warm state 也会保留。新控制
仍须等待当前 generation 的新鲜 Tracker 结果，但严格隔离 MPC 内部状态的实验应重启
桥接器进程。

控制还受到以下新鲜度条件限制：

- state 的本地接收年龄不超过 200 ms；
- 算法结果与当前 generation 一致；
- 源观测的墙钟接收年龄不超过 100 ms；
- 当前 state 的 `sim_time` 与 `planning_sim_time` 之差在 0 到 30 ms 内；
- Tracker 已经进入过 `tracking`；首次确认前的 `detecting → temp_lost` 不允许控制；
- 当前为 `tracking`，或者确认后的 `temp_lost` 预测；
- Tracker 测量年龄加上从 `source_sim_time` 外推到 `planning_sim_time` 的时间不超过
  100 ms；
- 仿真没有暂停，并且 IPC 外部控制仍启用。

开火在上述控制条件之外还要求当前 Tracker 严格为 `tracking`。确认后的短暂
`temp_lost` 可以继续预测控制，但不会继续给出开火许可。

计划结果同时保留三个时间概念：`source_sim_time` 是产生 Tracker 状态的观测时刻，
`planning_sim_time` 是 Planner 外推和求解使用的时刻，命令日志中的 `state_sim_time` 是
发布命令时主线程已知的最新状态时刻。

状态刚超过 200 ms 未更新时，运行循环只发布一次 `control=false, fire=false`，随后停止
写包，等待状态恢复或 3 秒超时报错。仿真暂停或 IPC 外部控制关闭时不发布任何包，避免
持续制造协议拒绝。退出或异常析构时，只有最近成功发布的指令仍为 `control=true`，并且
最近状态仍表示仿真接受命令，才会尽力再发停止控制；暂停和禁用状态不会发送析构停止包。

state 和 command 都是最新值邮箱，不是消息队列。状态 publication 和命令 seq 被跳过
是正常现象；判断闭环时应看最新 `ipc.last_seq`、实际云台姿态和拒绝计数。

## 输出文件

每次运行的输出目录包含：

| 文件 | 内容 |
| --- | --- |
| `run.json` | channel、控制/开火开关、输入模式、`estimator`、Detector 后端、配置路径、构建选项和源码版本 |
| `detector_config.yaml` | raw 模式实际加载的 YOLO 配置和绝对模型路径；exact 模式不生成 |
| `effective_config_<N>.yaml` | generation N 使用的实时标定、坐标约定和算法参数 |
| `observations.jsonl` | 每个已处理观测的完整 state/frame 元数据和算法诊断；不复制大体积 raw 像素 |
| `commands.jsonl` | 控制模式下运行循环实际发布成功的命令；dry-run 中为空文件 |
| `imgs/`、`patterns/` | 上游 Detector 创建的可选调试目录，统一限制在本次输出目录内 |

`observations.jsonl` 的 `result` 主要字段包括：

- 输入模式、generation、`source_sim_time`、`planning_sim_time`、tick 和总处理耗时；
- `detector_ms`、Detector 原始类别/颜色/置信度、四角点与可选传统修正角点；
- Detector/精确角点输入数量和实际交给 Tracker 的候选数量；
- Tracker 状态及本帧是否得到测量；
- `tracker_measurements` 中 Solver 的 `pnp_position_world/pnp_yaw_world`、实际进入 Target 的
  `used_position_world/used_yaw_world`，以及可用的 `innovation_ypda`；
- `target_age_at_plan_ms` 和 `measurement_age_at_plan_ms`；
- `armors` 中 Tracker 候选列表在处理后的位姿与 truth 评分；UV 初始化 hook 可能改写该列表，
  判断 Target 实际使用的测量应以 `tracker_measurements[].used_*` 为准；
- Planner 的控制意图、内部开火意图、目标板索引和完整控制轨迹。

每条结果顶层的 `estimator` 明确记录本帧走 `pnp` 还是 `uv-ekf`。UV 模式还包含
`uv_ekf` 诊断对象，主要字段为：

- `segment_id`、`status`、`selected_source_id`；
- 初始化、预测、更新、重置是否发生或成功；
- `measurement_ready`：当前 UV 结果是否可以作为 Target 测量；
- `measurement_injected`：override 是否已把该测量交给 Tracker 的 Target 更新路径；它不
  单独代表 Target EKF 已接受，后者还要检查 `measurement_found`；
- `pre_update_rmse_px`、`post_update_rmse_px` 和 `nis`；
- `state`、`covariance_diagonal` 和 `processing_ms`。

判断 UV 是否参与控制时，不能只看 `uv_ekf.initialized` 或存在状态值；应同时检查本帧
`measurement_ready`、`measurement_injected`、Tracker 是否接受测量以及最终控制新鲜度。
更新拒绝但保留预测状态的帧不会伪装成新测量。

观测日志中的计划在该观测的 `source_sim_time` 生成。观测之间的 100 Hz 重规划结果用于
实时命令，并通过下面的命令日志保存，不为同一 state 反复追加观测日志行。

每个 `commands.jsonl` 记录都对应一次确实写入邮箱的命令，包含准确的 `seq`、实际发布的
`control` 和 `fire`、Tracker 状态、`planner_fire_intent`，以及 `source_sim_time`、
`planning_sim_time`、`state_sim_time`。客户端可能因 state 已经过期把请求的控制和开火
降为 false，日志记录的是降级后的实际值。暂停或 IPC 禁用时没有记录，因为此时根本不
写包。程序退出时的尽力停止包不经过主循环，可能不出现在该日志中。

`ipc_feedback` 是发送该行指令时客户端已知的最近一次 state 反馈，并不是对同一行新指令
的同步确认。需要在后续 state 中检查 `ipc.last_seq`、`received`、`rejected` 和实际姿态。

`run.json` 中的版本字段为：

- `simulator_revision.commit` 和 `simulator_revision.dirty`；
- `autoaim_revision.root`、`autoaim_revision.commit` 和 `autoaim_revision.dirty`。

这些值在 CMake 配置阶段写入编译定义，不是进程启动时重新计算的源码哈希。源码或工作区
状态改变后应重新运行 CMake 配置，才能刷新 `run.json` 中的记录。`dirty` 只表示是否有
修改，不保存具体 diff；做对比实验时仍应保留完整构建命令和工作区差异。

当前诊断保存 Detector 输入角点、Tracker 所选测量的原始 PnP 与最终 `used_*` 位姿，以及
可用的真值误差，但没有保存原始 IPPE 多解、所选分支、分支重投影误差、精简自由度/BA
优化是否接受以及优化代价。因此 JSONL 可以评估检测角点和最终测量，却不能定位 IPPE
分支或 refinement 阶段的问题；比较这些阶段时需要另加上游 Solver 诊断。

## 验证

先运行不启动仿真的离线测试：

```bash
ctest \
  --test-dir .tools/auto-aim-bridge \
  --output-on-failure
```

`bridge_test` 覆盖角点与车型映射、非法输入拒绝、合成投影的 PnP 精度、truth 对计划输出
无影响、raw 模式不读取理想角点/车型标签、RGB/RGBA 邮箱复制与错误长度拒绝、图像尺寸
校验、确认门限、持续丢失和 generation 重启等行为。它还覆盖 UV-EKF 的 PnP 初始化、
像素更新注入、无候选时不回退 PnP、连续拒绝后的重置与重新 seed，以及新 generation
重新建立 UV 分段。

`simple_uv_ekf_test` 直接复用上游数学测试，覆盖大小装甲投影、数值 Jacobian、带畸变相机、
运动与不规则时间间隔、缺测、NIS/重投影拒绝、协方差半正定性和非法配置保护。

准备好 `.tools/godot` 后，可以运行真实无界面仿真的闭环 smoke test：

```bash
python3 auto_aim_bridge/tests/live_smoke.py \
  --godot "$PWD/.tools/godot" \
  --bridge "$PWD/.tools/auto-aim-bridge/auto_aim_bridge" \
  --suite=exact
```

`exact` suite 使用三个随机临时 channel，依次验证静止 dry-run、1 rad/s 自转目标控制，
以及静止目标显式开火。它检查：

- dry-run 不创建指令、不移动云台；
- PnP 位置最大误差小于 1 cm，yaw 最大误差小于 0.03 rad；
- Tracker 能反复确认目标，Planner 能产生有效控制；
- 命令 seq 严格递增，活动命令使用的计划不超过最新状态 30 ms；
- 旋转目标一旦进入活动控制便不应中断，最终 Tracker 和 Planner 仍保持有效；
- 控制指令被接收且未被拒绝，旋转场景的 yaw/pitch 跟随误差分别小于 0.10/0.04 rad；
- dry-run 和仅控制场景全程不开火；
- 开火场景只有在 tracking、Planner 意图和全部安全门限同时成立时发布 `fire=true`；
- 仿真实际出膛、命中并扣减血量，退出后没有残留待出膛事件。

真实图像 suite 需要图形桌面，不能在普通 `--headless` CI 中静默跳过：

```bash
python3 auto_aim_bridge/tests/live_smoke.py \
  --godot "$PWD/.tools/godot" \
  --bridge "$PWD/.tools/auto-aim-bridge/auto_aim_bridge" \
  --suite=raw
```

它分别运行 raw 静止 dry-run 和静止闭环控制，检查反复 YOLO 检测、四关键点、PnP、Tracker
确认、活动控制、命令时序和不开火安全门。检测/跟踪不稳定时允许控制按安全门撤销，再由
后续新鲜检测重新确认；测试仍要求足够多的有效控制命令和实际云台跟随。当前固定模型与
渲染场景的 smoke 基线中，最大关键点误差门限为 50 px；本次实现验证结果分别约为
2.05 px（dry-run）以及 4.36–6.59 px（闭环移动视角）。`--suite=all` 会依次运行两套测试。

live smoke 使用临时目录，成功或失败退出后都会清理场景配置和桥接日志。需要长期分析的
实验应使用前面的手动命令和独立输出目录。

## 当前限制

- `SimpleUvEkf` 是单装甲板恒速度/恒角速度近似，不是完整整车 UV 模型。板面切换、两个
  同类候选接近或高角速度遮挡时，可能出现关联歧义、重置和重新 PnP seed。
- UV 初始化时从 PnP 固定 pitch/roll，后续只估计单板位置和 yaw。目标存在明显俯仰、
  横滚或非刚性角点误差时，像素残差可能被错误分配到深度与 yaw。
- UV 输出仍要经过原有 Target/Tracker 才进入 Planner。这保留了整车几何与控制接口，
  但属于两级估计，当前不会把 UV 的完整协方差直接替代 Target 的测量噪声模型。
- UV 更新被拒绝时不会回退到当前帧 PnP；短时间内由 Target 预测继续控制，超过新鲜度门限
  则撤销控制。日志中出现较少的 PnP 结果不代表系统卡死，应先检查 `uv_ekf.status`、
  `measurement_ready`、`measurement_injected` 和重置次数。
- 现有静态或理想角点结果不能证明 UV-EKF 一定优于 PnP。正式比较至少应固定相同输入、
  标定、时间戳、场景和配置，并分别统计三维误差、像素残差、控制中断和命中率。
- 默认 `exact-corners` 仍是理想二维角点；要评价 Detector 必须显式使用 `raw-frame` 和
  图形模式。
- 默认 YOLO 模型来自当前 `AUTOAIM_ROOT`，检测结果受模型版本、渲染材质、曝光、目标
  尺寸和姿态影响。raw smoke 的像素阈值不是实车精度承诺。
- 上游 `auto_aim::Detector::detect(full_image)` 即使 `debug=false` 仍无条件调用
  `cv::imshow`，因此桥接器采用生产同步 `auto_aim::YOLO`；YOLOv5 内部的传统候选修角
  重载没有该显示问题。
- 当前只支持 1440 × 1080；Tracker 的候选排序仍使用固定中心 `(720, 540)`。
- PnP 假设小装甲 135 × 56 mm、大装甲 230 × 56 mm。若在仿真中修改灯条间距、长度、
  倾角或缩放，角点仍会反映新几何，但自瞄物点模型不会自动改变，PnP 将产生系统误差。
- Planner 只接受 10 到 25 m/s 的弹速。上游在范围外会静默替换成 22 m/s，因此桥接器
  直接拒绝范围外的 `weapon.speed`。
- 当前弹道器固定使用 9.8 m/s²，而仿真默认重力为 9.7833 m/s²。桥接器会注入
  `weapon.delay`，但 Planner 仍不读取 `weapon.gravity` 或 `weapon.barrel_length`。
- 默认 `air_resistance=0`，与仿真无空气阻力一致。枪口长度和实际云台伺服响应仍会造成
  Planner 模型与仿真弹丸之间的差异。
- 自动 smoke 的显式开火场景目前只覆盖 exact 模式。raw 模式可以手动增加
  `--control --allow-fire`，但 Detector 抖动、仿真命中率和实车命中率应分开评估。
- Solver 中的 world 是旋转稳定、以云台原点为参考的坐标，不包含 `gimbal.origin` 的全局
  平移。诊断与仿真 truth 比较时会先减去云台世界原点。
- 理想角点按朝向和视场产生，不等同于逐像素检测结果；命中灭灯或战亡时，相应角点可能
  暂时或持续消失。
- state 与 command 都可能跳过中间值，运行频率和日志行数不构成硬实时保证。
- generation 会重建 Solver 和 Tracker，但保留 Planner/TinyMPC warm state。
- 当前 PnP 日志只有最终位姿与真值误差，不含原始 IPPE 分支和 refinement 过程指标。

这些差异意味着桥接器可以验证仿真渲染到控制的完整软件链路，但不能单独证明真实相机
域的检测性能或实车精确弹着表现。

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| `Start the simulator first` | 先启动带 `--remote` 的仿真，确认 bridge 与仿真 channel 一致，并检查 `/dev/shm/rm_aim_sim_<channel>` |
| `Cannot lock client.lock` | 同一 channel 已有 `control_demo.py`、相机示例、另一个 bridge 或其他客户端；退出原客户端或换 channel |
| `Frame timeout: ... not --headless` | `raw-frame` 必须启动图形仿真；确认没有使用 `--headless`，并检查窗口渲染仍在运行 |
| 找不到模型或 OpenVINO | 检查 `detector_config.yaml`、`AUTOAIM_ROOT/assets` 和 `OpenVINO_DIR`；模型相对路径只按配置目录及 AUTOAIM_ROOT 解析 |
| 提示必须使用 1440 × 1080 | 在仿真相机设置或场景配置中恢复 1440 × 1080；只修改 YAML 内参不能绕过 Tracker 的固定中心 |
| `weapon.speed must be 10..25` | 将仿真弹速调回 Planner 支持范围；默认值 23 m/s 可直接使用 |
| `Unsupported appearance.robot` | exact 模式只支持英雄、步兵 3/4/5 和哨兵预设；raw 类别来自 Detector |
| 运行后没有控制 | dry-run 不发送指令；控制模式还需 `--remote`、新鲜 state、Tracker 曾进入 `tracking`、测量总年龄不超过 100 ms，且计划落后当前状态不超过 30 ms |
| raw 能检测但控制偶尔撤销 | 查看 `detector_ms`、`detections`、Tracker 状态和测量年龄；检测/PnP 抖动触发失配时，撤销旧控制是预期安全行为 |
| UV-EKF 已初始化但没有新测量 | 检查 `uv_ekf.status`、`selected_source_id`、NIS、更新前 RMSE、`measurement_ready` 和 `measurement_injected`；拒绝帧按设计 prediction-only，不会回退 PnP |
| 有控制但不开火 | 确认同时使用 `--control --allow-fire`，并检查命令日志中的 Tracker 状态、`planner_fire_intent` 和实际 `fire`；`temp_lost` 不允许开火 |
| 开火一段时间后停止 | 检查 `health.alive` 和 `stats.kills`；默认目标被击毁后不再提供理想角点，可恢复或提高 `target.max_health` |
| 控制偶尔变为 false | 检查 Tracker 是否完成过确认、`measurement_age_at_plan_ms`、处理耗时，以及命令的 source/planning/state 三个仿真时间 |
| PnP 距离或 yaw 有固定偏差 | 检查是否修改了仿真灯条几何、相机外参/FOV，确认角点仍为 TL/TR/BR/BL，并查看对应 effective config |
| 输出目录被拒绝 | `--output` 只能使用不存在或空目录；每次实验使用新的目录 |
| 3 秒后报 state 超时 | 仿真可能已退出、脚本错误导致停止发布、channel 不匹配、本机系统时间异常，或 state 文件不再产生新的 publication |
| CMake 找不到 g2o/fmt/spdlog | 安装对应开发包，避免混用 Conda 与系统库；非系统前缀通过 `CMAKE_PREFIX_PATH` 指定 |
