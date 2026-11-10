# 仿真数据集导出器

本目录提供一个独立、只读的数据集客户端。它复用 Godot 已有的
`request.json → frame-<request_id>.raw + frame.json` 链路，将严格同帧的 RGB 图像、
装甲灯条四角点、相机标定、云台姿态和场景元数据保存为模型无关的 canonical 数据集。

导出器不运行 Detector、PnP、Tracker 或 Planner，不读取 `truth` 作为算法输入，不控制云台，
不开火，也不会创建或修改 `command.json`。原始训练 PNG 永远不画标注；overlay 只写入
`previews/`。

## 环境与边界

- 仅支持本仓库已经支持的 Linux 同机 IPC。
- raw frame 必须在图形模式运行；`--headless` 没有 Sensor Viewport 像素。
- Python 工具仅使用标准库，不要求 NumPy、OpenCV 或 Pillow。
- 导出器 v1 为了和当前自瞄适配契约一致，严格要求 `1440×1080`。这是本导出器策略，
  不是 IPC 协议本身的限制；仿真 UI/协议可以使用其他分辨率。
- 同一个 channel 只能有一个客户端。导出器不能和 `camera_demo.py`、`control_demo.py`、
  `auto_aim_bridge` 或另一个导出器同时占用同一 channel；它使用现有 `client.lock` 互斥。
- 输出目录必须不存在或为空，避免覆盖已有数据。

## 快速开始

先在图形桌面启动仿真。默认场景示例：

```bash
./run.sh -- --remote
```

本目录还提供两个独立 smoke 场景。`smoke_mixed.cfg` 会让目标周期性离开视场，从而同时覆盖
有标签帧和空帧：

```bash
./run.sh -- --remote --config=res://tools/dataset_export/configs/smoke_mixed.cfg
```

另一个终端采集：

```bash
python3 tools/dataset_export/export_dataset.py \
  --channel=dataset_smoke_mixed \
  --output=/tmp/rm_aim_dataset_smoke_mixed_100 \
  --episode-id=smoke_mixed_001 \
  --fps=5 \
  --max-frames=100 \
  --image-format=png \
  --include-empty \
  --preview-every=10 \
  --config=res://tools/dataset_export/configs/smoke_mixed.cfg \
  --scene-id=dataset_smoke_mixed \
  --trajectory-id=wide_translate_spin \
  --random-seed=0
```

`--duration` 使用仿真时间秒；`--max-frames` 是最终成功保存的帧数。两者均为 0 时明确表示持续
运行到 Ctrl+C。采样依据 `sim_time`，只有相邻候选满足 `Δt >= 1/fps` 才会保存；不按墙钟
sleep 伪造帧，也不补齐丢帧。`--exclude-empty` 会在采样时隙排除空标签帧，默认
`--include-empty`。`--config` 只作为可复现性声明记录，导出器不会加载配置去控制仿真。

全部参数见：

```bash
python3 tools/dataset_export/export_dataset.py --help
```

## raw frame 生命周期

`dataset_protocol.StrictFrameClient` 扩展现有 Python `LocalClient`，但关闭时明确使用
`send_stop=False`，因此只写 `request.json`。每次取图执行：

1. 等待新的、新鲜的 `state.json`，记录 publisher、session 和静态 capture context；
2. 持有独占 `client.lock`，用临时文件加原子 rename 发布唯一 `request_id`；
3. 同一时刻只允许一个请求在途；
4. 等待 `frame.json` 同时匹配当前 request、publisher、session 和静态 context；
5. 检查 `protocol/type`、有限且新鲜的 `sim_time`、tick、`camera.timestamp/tick`、编码、
   宽高、文件名和精确字节数；
6. 用 `O_NOFOLLOW` 打开准确的 `frame-<request_id>.raw`，检查 regular file 与 `fstat` 大小，
   完整复制像素；
7. 完成 PNG、canonical、frame metadata 和可选 preview 的落盘后，才发送下一个请求。

服务端开始处理请求时便会消费该 request id；失败或超时后客户端必须产生新的 id，不能用旧 id
重试。publisher/session/static context 改变、tick 或 `sim_time` 回退时，当前 episode 明确停止；
不会把重启前后的帧静默拼在一起。重复 request、tick、仿真时间和已经保存过的图像内容会拒绝并
计数。

## 输出结构

```text
dataset/
├── dataset.json
├── classes.json
├── split.json                         # 运行 split_dataset.py 后生成
├── annotations/
│   ├── canonical.jsonl
│   └── coco_keypoints.json            # 运行转换器后生成
├── episodes/
│   └── smoke_mixed_001/
│       ├── images/frame_000000.png     # 干净 RGB 训练图
│       ├── annotations.jsonl           # 本 episode canonical 记录
│       ├── frames.jsonl                # 原始同帧 frame.json 元数据
│       └── previews/*_overlay.png      # 独立 overlay
└── reports/
    ├── export_summary.json
    └── validation.json
```

PNG 不 resize、不转 BGR、不做未记录的 Gamma、颜色、曝光或增强。`rgb8` 逐字节写成 RGB；
`rgba8` 只丢弃 alpha 后写 RGB。写完会立即重新解码并比较每个采样值和宽高。

`dataset.json` 记录创建时间、CLI、channel、publisher/session、Git commit 与 dirty 状态、Godot
版本、编码/分辨率、FPS、K/D/R/t、场景配置声明及 SHA256、seed、类别映射路径、统计和可见性
限制。dirty 状态只保存 `git status --short`，不会复制完整 diff。

## Canonical annotation

`annotations/canonical.jsonl` 是唯一真值来源。每一行对应且只对应一张图，主要结构为：

```json
{
  "schema_version": 1,
  "episode_id": "smoke_mixed_001",
  "frame_index": 0,
  "image": "episodes/smoke_mixed_001/images/frame_000000.png",
  "request_id": "...",
  "publisher_id": "...",
  "session_id": "...",
  "tick": 123,
  "sim_time": 1.25,
  "width": 1440,
  "height": 1080,
  "encoding": "RGB8",
  "source_encoding": "rgb8",
  "camera": {
    "K": [],
    "distortion": [],
    "R_camera2gimbal": [],
    "t_camera2gimbal": [],
    "gimbal_quaternion_wxyz": []
  },
  "appearance": {},
  "health": {},
  "gimbal": {},
  "weapon": {},
  "armors": []
}
```

每块装甲为：

```json
{
  "armor_id": 0,
  "team": "blue",
  "robot": "infantry_3",
  "robot_number": 3,
  "armor_type": "small",
  "corners_tl_tr_br_bl": [[0, 0], [1, 0], [1, 1], [0, 1]],
  "bbox_xyxy": [0, 0, 1, 1],
  "bbox_clipped_xyxy": [0, 0, 1, 1],
  "center_uv": [0.5, 0.5],
  "polygon_area_px2": 1,
  "truncated": false,
  "difficult": null,
  "occlusion": null,
  "visibility": "geometric_projection_only"
}
```

四角点顺序严格是 **TL, TR, BR, BL**，图像坐标为左上原点、u/x 向右、v/y 向下。导出器
信任协议语义并保持原顺序，只检查有限值、正面积、image-coordinate clockwise winding、
自交、bbox 和边界；不会用 x/y 排序破坏透视下的点身份。

`bbox_xyxy` 来自未裁剪角点 min/max；`bbox_clipped_xyxy` 另行裁到图像边界。离线导入的部分
出画点会原样保留并设置 `truncated=true`。不过当前 Godot 发布器只有四个灯条点全部在画内才
输出该装甲，所以 live `camera.armors` 的部分出画目标会整块缺席，正常 live 记录的
`truncated` 为 false。

frame 中存在的同帧 `appearance/health/gimbal/weapon/stats/ipc` 会原样保留；bbox、中心、面积、
truncated、robot number 和 armor type 明确标为推导字段。协议没有 confidence、逐板 health、
距离、三维位置、normal、armor yaw、visibility 或 occlusion，导出器不会从异步 `state.json`
猜测这些值，未知的 difficult/occlusion 使用 null。

## 可见性与遮挡语义

Godot 源码当前只把满足以下条件的装甲加入 `camera.armors`：

- 目标存活且该装甲灯条点亮；
- 装甲正面朝向相机；
- 四个灯条端点深度有效并全部位于图像内。

普通命中使对应灯条熄灭 0.05 仿真秒，此时该板不产生标签；战亡后所有灯条持续熄灭，标签
列表为空。完全出画和背向相机的板也不输出。

这里没有 raycast、render-depth 或逐像素遮挡测试。标签是几何投影，**不保证目标在图像中逐
像素可见**，不能称为完美 visibility/occlusion GT。板体仍由渲染器正常深度测试，因此图像和
理想标签在存在遮挡时可能不一致。

## 类别与当前 Detector 契约

Canonical 始终分别保存 `team`、`robot`、`robot_number`、`armor_type` 和物理
`armor_id (0..3)`；物理板 id 不是车型编号或网络类别。

| canonical robot | 机器人编号 | 当前 v5 num_id | canonical armor_type |
|---|---:|---:|---|
| `hero` | 1 | 1 | `large`（源码名 `big`） |
| `infantry_3` | 3 | 3 | `small` |
| `infantry_4` | 4 | 4 | `small` |
| `infantry_5` | 5 | 5 | `small` |
| `sentry` | 7 | 0 | `small` |

`armor_type` 是由 `appearance.robot` 的内置 preset 推导，不是 frame 原生字段。实际
`lightbar_spacing/lightbar_length` 位于场景配置且允许覆盖，却不随 frame 发布；自定义配置若
修改几何，仅凭数据帧无法证明真实尺寸。应始终传 `--config` 保存外部声明和哈希，且不要把
preset type 当成已经同步验证的物理尺寸。

只读审计相邻 `auto-aim-new-0` 后，当前配置实际使用自定义 YOLOv5：输出
`[1,25200,22] = 8 个关键点坐标 + 1 个 objectness logit + 4 个 color logits + 9 个 number
logits`。运行时颜色 head 为 `0=blue, 1=red, 2/3=extinguish`；网络四对点按 raw pair
`[0,3,2,1]` 重排成 Solver 要求的 TL/TR/BR/BL。它不是普通单 class-id YOLO bbox/pose。

相邻仓库没有训练脚本、dataset reader、loss/head 定义或模型导出契约，也缺少原训练标签的
raw pair/归一化/像素中心约定。因此本导出器不宣称任何输出能直接训练当前模型；完整审计结果和
映射保存在 `classes.json`。

## COCO Keypoints 转换

Canonical 正确后可离线生成通用 COCO Keypoints：

```bash
python3 tools/dataset_export/convert_coco_keypoints.py \
  --dataset=/tmp/rm_aim_dataset_smoke_mixed_100
```

转换器使用一个通用 `armor` category，keypoint 顺序仍为 TL/TR/BR/BL，类别字段保留在每条
annotation 的 `attributes`，不会强行把颜色和编号压成一个 class id。bbox 使用 clipped
`xywh`，segmentation 使用原始四边形。默认 keypoint visibility 为 1，表示“有标签，但不对
逐像素可见性作保证”；只有显式 `--assume-visible` 才写 2。该 JSON 是通用交换格式，**不能
直接声称兼容当前自定义 YOLOv5 训练**。

## Preview

```bash
python3 tools/dataset_export/preview_dataset.py \
  --dataset=/tmp/rm_aim_dataset_smoke_mixed_100 \
  --count=50 \
  --seed=7
```

TL/TR/BR/BL 分别用红、绿、蓝、黄圆点，紫色四边形、青色 bbox；同时显示 team、robot、
armor type、armor id、frame、tick 和 sim_time。输出写入 episode 的 `previews/`，并在写前后
校验原 PNG 哈希不变。

## Validation

```bash
python3 tools/dataset_export/validate_dataset.py \
  --dataset=/tmp/rm_aim_dataset_smoke_mixed_100 \
  --min-empty-ratio=0.05
```

Validator 检查严格 JSON/JSONL、路径不逃逸、PNG CRC/解码/尺寸、有限数值、四角顺序与面积、
自交、bbox/中心/面积/truncated 一致性、armor type 映射、重复 request/tick/image path/图像
内容、canonical/episode/frame 行数、图片数量、dataset totals、split episode 泄漏和指定空帧
比例。错误时退出码非零，结果写到 `reports/validation.json`。

## Episode 级划分

禁止在采集时把相邻帧随机拆到不同集合。采集先按 episode 保存，随后生成只含 manifest、
不复制图片的 split：

```bash
python3 tools/dataset_export/split_dataset.py \
  --dataset=/tmp/rm_aim_dataset_smoke_mixed_100 \
  --seed=20261110 \
  --train=0.70 --val=0.15 --test=0.15 \
  --group-by=episode
```

支持 `episode`、`trajectory`、`seed`、`session` 分组；同一 episode 始终是不可拆分原子，同一
trajectory/seed group 也会整体分配。随机种子必须显式提供。只有一个 episode 时它会完整进入
一个 split，不能凭空满足 70/15/15；真实划分需要采集多个独立 episode。

## 离线测试

测试不依赖正在运行的 Godot：

```bash
python3 -m unittest discover -s tools/dataset_export/tests -v
```

覆盖 RGB/RGBA、错误 raw 长度和编码、PNG 损坏、点序保留、bbox、部分出画、NaN/Inf、
自交/逆序、严格 frame metadata、重复 request/tick、session 改变、时间回退、split 泄漏、
canonical→COCO、validator 以及原图不含 overlay。

## 下一阶段

更大规模训练数据建议按 episode/seed 生成可复现的 domain randomization：材质、灯光、曝光、
背景、相机噪声、运动模糊、距离、车型/阵营和运动轨迹都应写入同步元数据。若要得到可靠的遮挡
标签，应先在 Godot 发布与 Sensor Viewport 同步的 depth/instance visibility，而不是把当前几何
投影当成逐像素真值。要训练现有自定义网络，还需取得原训练 reader/loss/export 契约并编写专用
converter。最终模型应混合真实标注图像做微调和独立实车验证；纯仿真指标不能代表实车性能。

