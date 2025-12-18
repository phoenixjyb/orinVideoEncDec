# orinVideoEncDec

[English](#english) | [中文](#中文)

## English

### Overview

This repo contains a ROS 2 Humble package for Jetson Orin that:

- Captures from V4L2 devices (GMSL cameras show up as `/dev/video*`)
- Encodes with Jetson NVENC (`nvv4l2h265enc`)
- Publishes H.265 access units as `sensor_msgs/msg/CompressedImage` (`format: "h265"`)

### Package

- `cr_h265_publisher`
  - Publisher node: `cr_h265_publisher_node`
  - Optional dump tool: `cr_h265_dump_node` (records ROS2 H.265 topic to a `.h265` file)

### Requirements (Orin)

- ROS 2 Humble installed at `/opt/ros/humble`
- Jetson GStreamer plugins:
  - `gst-inspect-1.0 nvv4l2h265enc`
  - `gst-inspect-1.0 nvvidconv`
  - If missing: `sudo apt-get install -y nvidia-l4t-gstreamer`

### Get repo onto Orin

Option A (recommended): clone on the Orin (this README uses `/home/nvidia/yanbo` on your Orin):

```bash
mkdir -p /home/nvidia/yanbo
cd /home/nvidia/yanbo
git clone https://github.com/phoenixjyb/orinVideoEncDec.git
```

Option B: copy only the ROS package from your dev machine:

```bash
ssh cr@192.168.100.150 'mkdir -p /home/nvidia/yanbo'
scp -r src/cr_h265_publisher cr@192.168.100.150:/home/nvidia/yanbo/
```

### Build on Orin

Option A (repo as a colcon workspace):

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/yanbo/orinVideoEncDec
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Option B (package copied into an existing workspace folder):

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/yanbo
colcon build --base-paths cr_h265_publisher --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Note: the rest of this README assumes you used Option A and your workspace is `/home/nvidia/yanbo/orinVideoEncDec`. If you used Option B or a different path, adjust the `source .../install/setup.bash` and `--params-file ...` paths accordingly.

### Camera init (GMSL)

If you use the existing camera init script (recommended), this will init cameras but NOT start `cr_camera_node`:

```bash
# Example if you have cr_camera_driver at /home/nvidia/yanbo/cr_camera_driver (adjust if yours is elsewhere)
bash /home/nvidia/yanbo/cr_camera_driver/scripts/start_cameras.sh --config-only --skip-build --high-performance
```

If you don’t know where it is on a new device:

```bash
find /home/nvidia -maxdepth 6 -name start_cameras.sh
```

It may prompt for sudo password (same as SSH password in your setup).

### Camera sanity check (device mapping + stream test)

On the Orin (no ROS needed). If you cloned this repo:

```bash
bash scripts/check_orin_cameras.sh
```

If you only copied the ROS package, copy this script too:

```bash
scp scripts/check_orin_cameras.sh cr@192.168.100.150:/tmp/check_orin_cameras.sh
bash /tmp/check_orin_cameras.sh
```

Optional env overrides:

```bash
DEVICES="/dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4" \
NUM_BUFFERS=5 MEASURE_BUFFERS=30 TIMEOUT_S=60 FPS=30 \
bash scripts/check_orin_cameras.sh
```

Interpreting results:

- If `content_check` reports `FLAT`, that `/dev/videoX` is likely producing a solid-color or frozen stream.
- On the current Orin setup we tested, `/dev/video2`, `/dev/video3`, `/dev/video4` produce non-flat images; `/dev/video0/1/5/6/7` were solid green.

### Run: publish H.265 over ROS2

Single camera example:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/yanbo/orinVideoEncDec/install/setup.bash

ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  -p devices:=[/dev/video2] \
  -p topics:=[/cr/camera/h265/cam2] \
  -p frame_ids:=[cam2] \
  -p input_format:=UYVY \
  -p output_width:=960 -p output_height:=768 \
  -p bitrate:=4000000
```

Use the provided params file:

```bash
ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  --params-file /home/nvidia/yanbo/orinVideoEncDec/install/cr_h265_publisher/share/cr_h265_publisher/config/params.yaml
```

Multiple cameras (recommended: 1 process per camera):

Some camera stacks (e.g. `v4l2-ctl -D` shows `Card type: vi-output, ...`) are unstable when you open multiple `/dev/video*` in ONE process (can show `not-negotiated` / Argus errors). In that case, use the launch file to run one node per camera:

```bash
ros2 launch cr_h265_publisher multi_cameras.launch.py \
  devices:=/dev/video2,/dev/video3,/dev/video4 \
  topics:=/cr/camera/h265/cam2,/cr/camera/h265/cam3,/cr/camera/h265/cam4 \
  frame_ids:=cam2,cam3,cam4 \
  input_format:=UYVY
```

### Encoder configuration (GOP / B-frames / rate control)

Check what your Orin supports (properties are provided by `nvv4l2h265enc`):

```bash
gst-inspect-1.0 nvv4l2h265enc | sed -n '/^Element Properties:/,/^Pad Templates:/p'
```

This project exposes the most common knobs as ROS parameters:

- GOP: `iframeinterval`, `idrinterval`
- Frame structure: `num_b_frames` (maps to `nvv4l2h265enc num-B-Frames`), `num_ref_frames`
- Rate control: `control_rate` (0=VBR, 1=CBR), `ratecontrol_enable`, `enable_twopass_cbr`, `peak_bitrate`, `vbv_size`

Examples:

- All-I (low latency, higher bitrate): `iframeinterval=1`, `idrinterval=1`, `num_b_frames=0`
- IP only (default): `num_b_frames=0`
- IPB (higher compression, adds latency): `num_b_frames=1` or `2`

Note: `gst-inspect` still prints “Supported only on Xavier” for `num-B-Frames`, but we verified the pipeline runs on Orin; treat it as optional and test on your exact JetPack/L4T build.

### Quick end-to-end validation (record + decode)

1) Start publisher in one terminal.

2) In another terminal, record ~300 messages to a raw H.265 bytestream:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/yanbo/orinVideoEncDec/install/setup.bash

ros2 run cr_h265_publisher cr_h265_dump_node --ros-args \
  -p topic:=/cr/camera/h265/cam2 \
  -p output:=/tmp/cam2.h265 \
  -p max_messages:=300
```

3) Decode the file (headless):

```bash
gst-launch-1.0 -q filesrc location=/tmp/cam2.h265 ! h265parse ! nvv4l2decoder ! fakesink sync=false
```

### CPU benchmark (scaling with camera count)

Copy the benchmark script to the Orin (or clone this repo there):

```bash
scp scripts/bench_orin_cpu.sh cr@192.168.100.150:/tmp/bench_orin_cpu.sh
```

Run on the Orin:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/yanbo/orinVideoEncDec/install/setup.bash
bash /tmp/bench_orin_cpu.sh
```

Notes:

- `proc_cpu(%)` is the publisher process CPU usage as “% of one core” (can exceed 100%).
- `sys_cpu(%)` is total system CPU usage as a % of all CPU cores combined (0–100).
- If you see camera failures at higher camera counts, try `MODE=multi_process` (one publisher process per camera).
- If `v4l2-ctl --get-fmt-video` shows `Pixel Format: 'YUYV'`, use `INPUT_FORMAT=YUY2` (or `YUYV`; it will be normalized).

Example result on the current Orin (requested `1920x1536@30` in, `960x768` out, `4Mbps`/cam, `DURATION_S=20`, devices `/dev/video0..4`):

```
cams   proc_cpu(%)  sys_cpu(%)
1      2.50         2.47
2      4.50         2.73
3      42.39        5.97
4      79.38        9.10
5      115.97       12.26
```

Example result using only the “fast” cameras (`DEV_INDICES='2 3 4'`, i.e. `/dev/video2..4`), Orin @ `192.168.100.150`:

```
cams   proc_cpu(%)  sys_cpu(%)
1      38.14        5.59
2      74.93        8.72
3      111.67       11.83
```

Example result on Orin @ `192.168.100.130` using `MODE=multi_process` and `INPUT_FORMAT=YUY2` (sum of per-camera processes):

```
cams   proc_cpu(%)  sys_cpu(%)
1      46.87        15.91
2      88.28        22.60
3      131.58       28.73
```

If you see a “big jump” when adding a specific camera index, it usually means that camera is actually running at a much higher FPS/throughput than the others (e.g. on this setup `/dev/video0` is ~1–2 fps while `/dev/video2` is near realtime).

### H.265 CPU vs hardware acceleration

- `nvv4l2h265enc` uses Jetson NVENC (hardware H.265 encoder).
- `nvvidconv` uses Jetson VIC (hardware color/scale convert into NVMM).
- CPU is still used for: GStreamer scheduling + pulling encoded buffers + copying bytes into ROS messages.
- Verify on Orin: run `sudo tegrastats --interval 1000` and watch `NVENC` / `VIC` activity.

### Notes / gotchas

- `cr_h265_publisher_node` opens `/dev/video*` directly.
  - Don’t run it at the same time as `cr_camera_driver` if they use the same `/dev/video*` devices.
- If you have mixed camera models / pixel formats, set `input_format` (global) or `input_formats` (per-device array).
- Each ROS message is one encoder output buffer (H.265 access unit, bytestream/Annex-B).

---

## 中文

### 功能概览

本仓库提供一个适用于 Jetson Orin（ROS 2 Humble）的编码发布包：

- 直接从 V4L2 设备采集（GMSL 相机通常是 `/dev/video*`）
- 使用 Jetson NVENC 硬件编码（`nvv4l2h265enc`）
- 将 H.265 码流按“访问单元”(Access Unit) 打包成 `sensor_msgs/msg/CompressedImage` 并发布（`format: "h265"`）

### 包内容

- `cr_h265_publisher`
  - 发布节点：`cr_h265_publisher_node`
  - 可选工具：`cr_h265_dump_node`（订阅 ROS2 的 H.265 topic 并保存为 `.h265` 文件）

### Orin 端依赖

- 已安装 ROS 2 Humble（`/opt/ros/humble`）
- Jetson GStreamer 插件可用：
  - `gst-inspect-1.0 nvv4l2h265enc`
  - `gst-inspect-1.0 nvvidconv`
  - 如果缺失：`sudo apt-get install -y nvidia-l4t-gstreamer`

### 将仓库放到 Orin 上

方式 A（推荐）：直接在 Orin 上 clone：

```bash
mkdir -p /home/nvidia/yanbo
cd /home/nvidia/yanbo
git clone https://github.com/phoenixjyb/orinVideoEncDec.git
```

方式 B：从开发机只拷贝 ROS 包：

```bash
ssh cr@192.168.100.150 'mkdir -p /home/nvidia/yanbo'
scp -r src/cr_h265_publisher cr@192.168.100.150:/home/nvidia/yanbo/
```

### Orin 端编译

方式 A（把本仓库当成 colcon workspace）：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/yanbo/orinVideoEncDec
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

方式 B（只拷贝了 ROS 包到现有 workspace 目录）：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/yanbo
colcon build --base-paths cr_h265_publisher --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

说明：本 README 后续命令默认你使用了方式 A（workspace 在 `/home/nvidia/yanbo/orinVideoEncDec`）。如果你使用了方式 B 或放在其它路径，请相应修改 `source .../install/setup.bash` 和 `--params-file ...` 的路径。

### 相机初始化（GMSL）

推荐使用现有脚本做相机初始化（只初始化，不启动 `cr_camera_node`）：

```bash
# 示例：如果你的 cr_camera_driver 在 /home/nvidia/yanbo/cr_camera_driver（如果路径不同请自行调整）
bash /home/nvidia/yanbo/cr_camera_driver/scripts/start_cameras.sh --config-only --skip-build --high-performance
```

如果你在新设备上不清楚脚本路径：

```bash
find /home/nvidia -maxdepth 6 -name start_cameras.sh
```

该脚本可能会提示输入 sudo 密码（你的环境里与 SSH 密码一致）。

### 相机连通性自检（设备映射 + 码流测试）

在 Orin 上运行（不依赖 ROS）。如果你已经 clone 了本仓库：

```bash
bash scripts/check_orin_cameras.sh
```

如果你只拷贝了 ROS 包，也可以单独把脚本拷过去：

```bash
scp scripts/check_orin_cameras.sh cr@192.168.100.150:/tmp/check_orin_cameras.sh
bash /tmp/check_orin_cameras.sh
```

可选环境变量：

```bash
DEVICES="/dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4" \
NUM_BUFFERS=5 MEASURE_BUFFERS=30 TIMEOUT_S=60 FPS=30 \
bash scripts/check_orin_cameras.sh
```

结果解读：

- 如果 `content_check` 输出 `FLAT`，通常意味着该 `/dev/videoX` 只有纯色或冻结画面。
- 在我们当前测试的 Orin 上，`/dev/video2`、`/dev/video3`、`/dev/video4` 有正常画面；`/dev/video0/1/5/6/7` 为纯绿画面。

### 运行：发布 H.265 ROS2 Topic

单路相机示例：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/yanbo/orinVideoEncDec/install/setup.bash

ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  -p devices:=[/dev/video2] \
  -p topics:=[/cr/camera/h265/cam2] \
  -p frame_ids:=[cam2] \
  -p input_format:=UYVY \
  -p output_width:=960 -p output_height:=768 \
  -p bitrate:=4000000
```

使用自带参数文件：

```bash
ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  --params-file /home/nvidia/yanbo/orinVideoEncDec/install/cr_h265_publisher/share/cr_h265_publisher/config/params.yaml
```

多路相机（推荐：每路 1 个进程）：

部分相机栈（例如 `v4l2-ctl -D` 显示 `Card type: vi-output, ...`）在同一进程内打开多个 `/dev/video*` 可能不稳定（会出现 `not-negotiated` / Argus 报错）。此时建议用 launch 文件为每路相机启动一个发布进程：

```bash
ros2 launch cr_h265_publisher multi_cameras.launch.py \
  devices:=/dev/video2,/dev/video3,/dev/video4 \
  topics:=/cr/camera/h265/cam2,/cr/camera/h265/cam3,/cr/camera/h265/cam4 \
  frame_ids:=cam2,cam3,cam4 \
  input_format:=UYVY
```

### 编码配置（GOP / B 帧 / 码率控制）

先查看你的 Orin 支持哪些编码参数（这些参数来自 `nvv4l2h265enc`）：

```bash
gst-inspect-1.0 nvv4l2h265enc | sed -n '/^Element Properties:/,/^Pad Templates:/p'
```

本项目将常用参数做成了 ROS 参数：

- GOP：`iframeinterval`、`idrinterval`
- 帧结构：`num_b_frames`（对应 `nvv4l2h265enc num-B-Frames`）、`num_ref_frames`
- 码率控制：`control_rate`（0=VBR，1=CBR）、`ratecontrol_enable`、`enable_twopass_cbr`、`peak_bitrate`、`vbv_size`

常见用法示例：

- 全 I 帧（低延迟但码率更高）：`iframeinterval=1`、`idrinterval=1`、`num_b_frames=0`
- 仅 IP（默认）：`num_b_frames=0`
- IPB（压缩率更高但会增加延迟）：`num_b_frames=1` 或 `2`

注意：`gst-inspect` 对 `num-B-Frames` 仍打印“仅 Xavier 支持”，但我们已在 Orin 上验证 pipeline 可运行；是否真正启用 B 帧请以你当前 JetPack/L4T 版本实测为准。

### 端到端验证（录制 + 解码）

1）一个终端启动发布节点。

2）另一个终端录制约 300 帧到 H.265 裸码流文件：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/yanbo/orinVideoEncDec/install/setup.bash

ros2 run cr_h265_publisher cr_h265_dump_node --ros-args \
  -p topic:=/cr/camera/h265/cam2 \
  -p output:=/tmp/cam2.h265 \
  -p max_messages:=300
```

3）使用硬件解码验证（无显示）：

```bash
gst-launch-1.0 -q filesrc location=/tmp/cam2.h265 ! h265parse ! nvv4l2decoder ! fakesink sync=false
```

### CPU 压力测试（相机数量扩展）

将测试脚本拷贝到 Orin（或者直接把本仓库放到 Orin 上）：

```bash
scp scripts/bench_orin_cpu.sh cr@192.168.100.150:/tmp/bench_orin_cpu.sh
```

在 Orin 上运行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/yanbo/orinVideoEncDec/install/setup.bash
bash /tmp/bench_orin_cpu.sh
```

说明：

- `proc_cpu(%)`：发布进程的 CPU 占用（按“单核百分比”计算，因此可能 >100%）。
- `sys_cpu(%)`：整机总 CPU 占用（按“所有 CPU 核总和”的百分比计算，范围 0–100）。
- 如果相机数量增加后出现失败，可以尝试 `MODE=multi_process`（每路相机一个发布进程）。
- 如果 `v4l2-ctl --get-fmt-video` 显示 `Pixel Format: 'YUYV'`，请使用 `INPUT_FORMAT=YUY2`（或 `YUYV`，会自动归一化为 `YUY2`）。

当前 Orin 上的一组示例结果（输入请求 `1920x1536@30`，输出 `960x768`，每路 `4Mbps`，`DURATION_S=20`，设备 `/dev/video0..4`）：

```
cams   proc_cpu(%)  sys_cpu(%)
1      2.50         2.47
2      4.50         2.73
3      42.39        5.97
4      79.38        9.10
5      115.97       12.26
```

仅使用“高吞吐量”的三路相机（`DEV_INDICES='2 3 4'`，即 `/dev/video2..4`），Orin @ `192.168.100.150` 的一组示例结果：

```
cams   proc_cpu(%)  sys_cpu(%)
1      38.14        5.59
2      74.93        8.72
3      111.67       11.83
```

Orin @ `192.168.100.130` 使用 `MODE=multi_process` 且 `INPUT_FORMAT=YUY2` 的示例结果（按每路进程求和）：

```
cams   proc_cpu(%)  sys_cpu(%)
1      46.87        15.91
2      88.28        22.60
3      131.58       28.73
```

如果你发现“加某一路之后 CPU 突然跳升”，通常意味着该相机实际帧率/吞吐量明显更高（例如当前环境里 `/dev/video0` 只有 ~1–2 fps，而 `/dev/video2` 接近实时）。

### H.265 编码是否用到 CPU？

- `nvv4l2h265enc`：使用 Jetson NVENC（硬件 H.265 编码器）。
- `nvvidconv`：使用 Jetson VIC（硬件颜色空间/缩放转换并输出 NVMM）。
- CPU 仍会参与：GStreamer 调度 + appsink 拉取编码数据 + 将字节拷贝进 ROS 消息。
- 验证方式：在 Orin 上运行 `sudo tegrastats --interval 1000`，观察 `NVENC` / `VIC` 的使用情况。

### 注意事项

- `cr_h265_publisher_node` 会直接打开 `/dev/video*`。
  - 如果 `cr_camera_driver` 也在使用同一设备，请不要同时运行（会抢占设备）。
- 如果你有不同型号/不同像素格式的相机，请设置 `input_format`（全局）或 `input_formats`（按设备逐一配置）。
- 每条 ROS 消息对应一次编码器输出（H.265 Access Unit，Annex-B/bytestream）。
