# orinEnc

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

### Deploy to Orin

From your dev machine:

```bash
scp -r src/cr_h265_publisher cr@192.168.100.150:/home/nvidia/wkp/
```

### Build on Orin

On the Orin:

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/wkp
colcon build --base-paths cr_h265_publisher --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Camera init (GMSL)

If you use the existing camera init script (recommended), this will init cameras but NOT start `cr_camera_node`:

```bash
bash /home/nvidia/wkp/cr_camera_driver/scripts/start_cameras.sh --config-only --skip-build --high-performance
```

It may prompt for sudo password (same as SSH password in your setup).

### Camera sanity check (device mapping + stream test)

On the Orin (no ROS needed):

```bash
bash scripts/check_orin_cameras.sh
```

Optional env overrides:

```bash
DEVICES="/dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4" \
NUM_BUFFERS=5 MEASURE_BUFFERS=30 TIMEOUT_S=60 FPS=30 \
bash scripts/check_orin_cameras.sh
```

### Run: publish H.265 over ROS2

Single camera example:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/install/setup.bash

ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  -p devices:=[/dev/video0] \
  -p topics:=[/cr/camera/h265/front_right] \
  -p frame_ids:=[front_right] \
  -p output_width:=960 -p output_height:=768 \
  -p bitrate:=4000000
```

Use the provided params file:

```bash
ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  --params-file /home/nvidia/wkp/install/cr_h265_publisher/share/cr_h265_publisher/config/params.yaml
```

### Quick end-to-end validation (record + decode)

1) Start publisher in one terminal.

2) In another terminal, record ~300 messages to a raw H.265 bytestream:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/install/setup.bash

ros2 run cr_h265_publisher cr_h265_dump_node --ros-args \
  -p topic:=/cr/camera/h265/front_right \
  -p output:=/tmp/front_right.h265 \
  -p max_messages:=300
```

3) Decode the file (headless):

```bash
gst-launch-1.0 -q filesrc location=/tmp/front_right.h265 ! h265parse ! nvv4l2decoder ! fakesink sync=false
```

### CPU benchmark (scaling with camera count)

Copy the benchmark script to the Orin (or clone this repo there):

```bash
scp scripts/bench_orin_cpu.sh cr@192.168.100.150:/tmp/bench_orin_cpu.sh
```

Run on the Orin:

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/install/setup.bash
bash /tmp/bench_orin_cpu.sh
```

Notes:

- `proc_cpu(%)` is the publisher process CPU usage as “% of one core” (can exceed 100%).
- `sys_cpu(%)` is total system CPU usage as a % of all CPU cores combined (0–100).

Example result on the current Orin (requested `1920x1536@30` in, `960x768` out, `4Mbps`/cam, `DURATION_S=20`, devices `/dev/video0..4`):

```
cams   proc_cpu(%)  sys_cpu(%)
1      2.50         2.47
2      4.50         2.73
3      42.39        5.97
4      79.38        9.10
5      115.97       12.26
```

Example result using only the “fast” cameras (`DEV_INDICES='2 3 4'`, i.e. `/dev/video2..4`):

```
cams   proc_cpu(%)  sys_cpu(%)
1      38.24        5.55
2      74.78        8.57
3      111.47       11.81
```

If you see a “big jump” when adding a specific camera index, it usually means that camera is actually running at a much higher FPS/throughput than the others (e.g. on this setup `/dev/video0` is ~1–2 fps while `/dev/video2` is near realtime).

### Notes / gotchas

- `cr_h265_publisher_node` opens `/dev/video*` directly.
  - Don’t run it at the same time as `cr_camera_driver` if they use the same `/dev/video*` devices.
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

### 部署到 Orin

在开发机上执行：

```bash
scp -r src/cr_h265_publisher cr@192.168.100.150:/home/nvidia/wkp/
```

### Orin 端编译

在 Orin 上执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/wkp
colcon build --base-paths cr_h265_publisher --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 相机初始化（GMSL）

推荐使用现有脚本做相机初始化（只初始化，不启动 `cr_camera_node`）：

```bash
bash /home/nvidia/wkp/cr_camera_driver/scripts/start_cameras.sh --config-only --skip-build --high-performance
```

该脚本可能会提示输入 sudo 密码（你的环境里与 SSH 密码一致）。

### 相机连通性自检（设备映射 + 码流测试）

在 Orin 上运行（不依赖 ROS）：

```bash
bash scripts/check_orin_cameras.sh
```

可选环境变量：

```bash
DEVICES="/dev/video0 /dev/video1 /dev/video2 /dev/video3 /dev/video4" \
NUM_BUFFERS=5 MEASURE_BUFFERS=30 TIMEOUT_S=60 FPS=30 \
bash scripts/check_orin_cameras.sh
```

### 运行：发布 H.265 ROS2 Topic

单路相机示例：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/install/setup.bash

ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  -p devices:=[/dev/video0] \
  -p topics:=[/cr/camera/h265/front_right] \
  -p frame_ids:=[front_right] \
  -p output_width:=960 -p output_height:=768 \
  -p bitrate:=4000000
```

使用自带参数文件：

```bash
ros2 run cr_h265_publisher cr_h265_publisher_node --ros-args \
  --params-file /home/nvidia/wkp/install/cr_h265_publisher/share/cr_h265_publisher/config/params.yaml
```

### 端到端验证（录制 + 解码）

1）一个终端启动发布节点。

2）另一个终端录制约 300 帧到 H.265 裸码流文件：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/install/setup.bash

ros2 run cr_h265_publisher cr_h265_dump_node --ros-args \
  -p topic:=/cr/camera/h265/front_right \
  -p output:=/tmp/front_right.h265 \
  -p max_messages:=300
```

3）使用硬件解码验证（无显示）：

```bash
gst-launch-1.0 -q filesrc location=/tmp/front_right.h265 ! h265parse ! nvv4l2decoder ! fakesink sync=false
```

### CPU 压力测试（相机数量扩展）

将测试脚本拷贝到 Orin（或者直接把本仓库放到 Orin 上）：

```bash
scp scripts/bench_orin_cpu.sh cr@192.168.100.150:/tmp/bench_orin_cpu.sh
```

在 Orin 上运行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/wkp/install/setup.bash
bash /tmp/bench_orin_cpu.sh
```

说明：

- `proc_cpu(%)`：发布进程的 CPU 占用（按“单核百分比”计算，因此可能 >100%）。
- `sys_cpu(%)`：整机总 CPU 占用（按“所有 CPU 核总和”的百分比计算，范围 0–100）。

当前 Orin 上的一组示例结果（输入请求 `1920x1536@30`，输出 `960x768`，每路 `4Mbps`，`DURATION_S=20`，设备 `/dev/video0..4`）：

```
cams   proc_cpu(%)  sys_cpu(%)
1      2.50         2.47
2      4.50         2.73
3      42.39        5.97
4      79.38        9.10
5      115.97       12.26
```

仅使用“高吞吐量”的三路相机（`DEV_INDICES='2 3 4'`，即 `/dev/video2..4`）的一组示例结果：

```
cams   proc_cpu(%)  sys_cpu(%)
1      38.24        5.55
2      74.78        8.57
3      111.47       11.81
```

如果你发现“加某一路之后 CPU 突然跳升”，通常意味着该相机实际帧率/吞吐量明显更高（例如当前环境里 `/dev/video0` 只有 ~1–2 fps，而 `/dev/video2` 接近实时）。

### 注意事项

- `cr_h265_publisher_node` 会直接打开 `/dev/video*`。
  - 如果 `cr_camera_driver` 也在使用同一设备，请不要同时运行（会抢占设备）。
- 每条 ROS 消息对应一次编码器输出（H.265 Access Unit，Annex-B/bytestream）。
