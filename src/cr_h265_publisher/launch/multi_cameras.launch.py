from __future__ import annotations

from dataclasses import dataclass

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _split_csv(value: str) -> list[str]:
    return [part.strip() for part in value.split(",") if part.strip()]


@dataclass(frozen=True)
class _CommonParams:
    input_width: int
    input_height: int
    output_width: int
    output_height: int
    fps: int
    control_rate: int
    ratecontrol_enable: bool
    enable_twopass_cbr: bool
    bitrate: int
    peak_bitrate: int
    vbv_size: int
    num_b_frames: int
    num_ref_frames: int
    iframeinterval: int
    idrinterval: int
    insert_sps_pps: bool
    insert_aud: bool
    maxperf_enable: bool
    preset_level: int
    profile: int


def _as_bool(value: str) -> bool:
    value_l = value.strip().lower()
    if value_l in ("1", "true", "yes", "on"):
        return True
    if value_l in ("0", "false", "no", "off"):
        return False
    raise ValueError(f"Invalid boolean: {value}")


def _generate_nodes(context, *args, **kwargs):
    devices = _split_csv(LaunchConfiguration("devices").perform(context))
    if not devices:
        raise RuntimeError("Launch arg 'devices' must be non-empty (comma-separated)")

    topics = _split_csv(LaunchConfiguration("topics").perform(context))
    if not topics:
        topics = [f"/cr/camera/h265/cam{i}" for i in range(len(devices))]
    if len(topics) != len(devices):
        raise RuntimeError("Launch arg 'topics' must match 'devices' length (or be empty)")

    frame_ids = _split_csv(LaunchConfiguration("frame_ids").perform(context))
    if not frame_ids:
        frame_ids = [f"cam{i}" for i in range(len(devices))]
    if len(frame_ids) != len(devices):
        raise RuntimeError("Launch arg 'frame_ids' must match 'devices' length (or be empty)")

    input_format = LaunchConfiguration("input_format").perform(context)
    input_formats = _split_csv(LaunchConfiguration("input_formats").perform(context))
    if input_formats and len(input_formats) != len(devices):
        raise RuntimeError("Launch arg 'input_formats' must match 'devices' length (or be empty)")

    common = _CommonParams(
        input_width=int(LaunchConfiguration("input_width").perform(context)),
        input_height=int(LaunchConfiguration("input_height").perform(context)),
        output_width=int(LaunchConfiguration("output_width").perform(context)),
        output_height=int(LaunchConfiguration("output_height").perform(context)),
        fps=int(LaunchConfiguration("fps").perform(context)),
        control_rate=int(LaunchConfiguration("control_rate").perform(context)),
        ratecontrol_enable=_as_bool(LaunchConfiguration("ratecontrol_enable").perform(context)),
        enable_twopass_cbr=_as_bool(LaunchConfiguration("enable_twopass_cbr").perform(context)),
        bitrate=int(LaunchConfiguration("bitrate").perform(context)),
        peak_bitrate=int(LaunchConfiguration("peak_bitrate").perform(context)),
        vbv_size=int(LaunchConfiguration("vbv_size").perform(context)),
        num_b_frames=int(LaunchConfiguration("num_b_frames").perform(context)),
        num_ref_frames=int(LaunchConfiguration("num_ref_frames").perform(context)),
        iframeinterval=int(LaunchConfiguration("iframeinterval").perform(context)),
        idrinterval=int(LaunchConfiguration("idrinterval").perform(context)),
        insert_sps_pps=_as_bool(LaunchConfiguration("insert_sps_pps").perform(context)),
        insert_aud=_as_bool(LaunchConfiguration("insert_aud").perform(context)),
        maxperf_enable=_as_bool(LaunchConfiguration("maxperf_enable").perform(context)),
        preset_level=int(LaunchConfiguration("preset_level").perform(context)),
        profile=int(LaunchConfiguration("profile").perform(context)),
    )

    respawn = _as_bool(LaunchConfiguration("respawn").perform(context))
    respawn_delay = float(LaunchConfiguration("respawn_delay").perform(context))

    nodes: list[Node] = []
    for i, device in enumerate(devices):
        fmt = input_formats[i] if input_formats else input_format
        params = {
            "devices": [device],
            "topics": [topics[i]],
            "frame_ids": [frame_ids[i]],
            "input_format": fmt,
            "input_width": common.input_width,
            "input_height": common.input_height,
            "output_width": common.output_width,
            "output_height": common.output_height,
            "fps": common.fps,
            "control_rate": common.control_rate,
            "ratecontrol_enable": common.ratecontrol_enable,
            "enable_twopass_cbr": common.enable_twopass_cbr,
            "bitrate": common.bitrate,
            "peak_bitrate": common.peak_bitrate,
            "vbv_size": common.vbv_size,
            "num_b_frames": common.num_b_frames,
            "num_ref_frames": common.num_ref_frames,
            "iframeinterval": common.iframeinterval,
            "idrinterval": common.idrinterval,
            "insert_sps_pps": common.insert_sps_pps,
            "insert_aud": common.insert_aud,
            "maxperf_enable": common.maxperf_enable,
            "preset_level": common.preset_level,
            "profile": common.profile,
        }
        nodes.append(
            Node(
                package="cr_h265_publisher",
                executable="cr_h265_publisher_node",
                name=f"cr_h265_publisher_cam{i}",
                output="screen",
                parameters=[params],
                respawn=respawn,
                respawn_delay=respawn_delay,
            )
        )

    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "devices",
                default_value="/dev/video2,/dev/video3,/dev/video4",
                description="Comma-separated V4L2 device paths (/dev/video*).",
            ),
            DeclareLaunchArgument(
                "topics",
                default_value="",
                description="Comma-separated topics (optional; defaults to /cr/camera/h265/camN).",
            ),
            DeclareLaunchArgument(
                "frame_ids",
                default_value="",
                description="Comma-separated frame_ids (optional; defaults to camN).",
            ),
            DeclareLaunchArgument(
                "input_format",
                default_value="UYVY",
                description="Input format for all cameras (GStreamer caps format, e.g. UYVY, YUY2).",
            ),
            DeclareLaunchArgument(
                "input_formats",
                default_value="",
                description="Comma-separated per-camera input formats (optional; overrides input_format).",
            ),
            DeclareLaunchArgument("input_width", default_value="1920"),
            DeclareLaunchArgument("input_height", default_value="1536"),
            DeclareLaunchArgument("output_width", default_value="960"),
            DeclareLaunchArgument("output_height", default_value="768"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("control_rate", default_value="1"),
            DeclareLaunchArgument("ratecontrol_enable", default_value="true"),
            DeclareLaunchArgument("enable_twopass_cbr", default_value="false"),
            DeclareLaunchArgument("bitrate", default_value="8000000"),
            DeclareLaunchArgument("peak_bitrate", default_value="0"),
            DeclareLaunchArgument("vbv_size", default_value="0"),
            DeclareLaunchArgument("num_b_frames", default_value="0"),
            DeclareLaunchArgument("num_ref_frames", default_value="1"),
            DeclareLaunchArgument("iframeinterval", default_value="30"),
            DeclareLaunchArgument("idrinterval", default_value="30"),
            DeclareLaunchArgument("insert_sps_pps", default_value="true"),
            DeclareLaunchArgument("insert_aud", default_value="false"),
            DeclareLaunchArgument("maxperf_enable", default_value="true"),
            DeclareLaunchArgument("preset_level", default_value="1"),
            DeclareLaunchArgument("profile", default_value="0"),
            DeclareLaunchArgument(
                "respawn",
                default_value="false",
                description="Respawn each camera node if it crashes.",
            ),
            DeclareLaunchArgument(
                "respawn_delay",
                default_value="2.0",
                description="Seconds to wait before respawning.",
            ),
            OpaqueFunction(function=_generate_nodes),
        ]
    )
