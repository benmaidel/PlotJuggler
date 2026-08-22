#!/usr/bin/env python3
# Copyright 2026 Davide Faconti
# SPDX-License-Identifier: MPL-2.0
"""Generate the synthetic MCAP fixture used by the headless 3D screenshot harness.

The fixture is the smallest recording that exercises the two data paths a 3D
scene needs — a TF tree and a dense point cloud — so a screenshot taken by
``screenshot_3d.sh`` is a meaningful visual check of the renderer:

  ``/tf``     ``tf2_msgs/msg/TFMessage``     world -> base_link -> sensor,
              20 Hz for 5 s. ``base_link`` drives a circle around the origin
              while yawing along its heading; ``sensor`` sits above it and
              spins, so the axis triads both translate and rotate.
  ``/points`` ``sensor_msgs/msg/PointCloud2``  a 2000-point spiral shell in the
              ``sensor`` frame, 5 Hz for 5 s. Fields are x/y/z/intensity, all
              FLOAT32 (``PointField.datatype == 7``), ``point_step`` 16.

Encoding conventions match the sibling generator in pj-official-plugins
(``data_load_mcap/test_data/generate_verification_mcaps.py``): schemas are
registered with encoding ``ros2msg`` (the concatenated ``.msg`` text ROS 2
records embed) and messages with encoding ``cdr``.

CDR notes that are easy to get wrong: every primitive is aligned relative to the
START OF THE PAYLOAD, i.e. after the 4-byte encapsulation header — so the
alignment base is offset 4, not 0. Strings carry a uint32 length that INCLUDES
the terminating NUL. Sequences carry a uint32 element count.

The cloud is deliberately small (~2000 points x 25 frames, zstd-chunked by the
MCAP writer) so the fixture stays well under 1 MB and can be regenerated in
under a second rather than committed as a binary.

Usage:
    generate_scene3d_fixture.py [OUTPUT.mcap]

Default output: ``<repo>/build/scene3d_fixture.mcap`` (a build artifact, never
committed). Requires: ``pip3 install --break-system-packages mcap``.
"""

from __future__ import annotations

import math
import struct
import sys
from pathlib import Path

from mcap.writer import Writer

# ---------------------------------------------------------------------------
# CDR writer
# ---------------------------------------------------------------------------


class CdrWriter:
    """Little-endian ROS 2 CDR (XCDR1) encoder.

    Emits the 4-byte encapsulation header up front and aligns every subsequent
    primitive relative to the byte AFTER that header, which is what the ROS 2
    CDR alignment rules require.
    """

    #: Bytes of encapsulation preceding the payload; also the alignment base.
    HEADER = b"\x00\x01\x00\x00"

    def __init__(self) -> None:
        self._buf = bytearray(self.HEADER)

    def _align(self, size: int) -> None:
        # Alignment is measured from the payload start, hence the -len(HEADER).
        while (len(self._buf) - len(self.HEADER)) % size != 0:
            self._buf.append(0)

    def uint8(self, value: int) -> "CdrWriter":
        self._buf.append(value & 0xFF)
        return self

    def boolean(self, value: bool) -> "CdrWriter":
        return self.uint8(1 if value else 0)

    def int32(self, value: int) -> "CdrWriter":
        self._align(4)
        self._buf.extend(struct.pack("<i", value))
        return self

    def uint32(self, value: int) -> "CdrWriter":
        self._align(4)
        self._buf.extend(struct.pack("<I", value))
        return self

    def float32(self, value: float) -> "CdrWriter":
        self._align(4)
        self._buf.extend(struct.pack("<f", value))
        return self

    def float64(self, value: float) -> "CdrWriter":
        self._align(8)
        self._buf.extend(struct.pack("<d", value))
        return self

    def string(self, value: str) -> "CdrWriter":
        raw = value.encode("utf-8") + b"\x00"
        self.uint32(len(raw))  # length INCLUDES the NUL
        self._buf.extend(raw)
        return self

    def uint8_sequence(self, raw: bytes) -> "CdrWriter":
        self.uint32(len(raw))
        self._buf.extend(raw)
        return self

    def sequence_length(self, count: int) -> "CdrWriter":
        return self.uint32(count)

    def bytes(self) -> bytes:
        return bytes(self._buf)


def write_header(cdr: CdrWriter, stamp_ns: int, frame_id: str) -> None:
    """std_msgs/Header: builtin_interfaces/Time stamp + string frame_id."""
    cdr.int32(stamp_ns // 1_000_000_000)
    cdr.uint32(stamp_ns % 1_000_000_000)
    cdr.string(frame_id)


# ---------------------------------------------------------------------------
# Schemas (concatenated ros2msg text, as ROS 2 bags embed it)
# ---------------------------------------------------------------------------

SCHEMA_TF_MESSAGE = b"""geometry_msgs/TransformStamped[] transforms
================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
Transform transform
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Transform
Vector3 translation
Quaternion rotation
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
"""

SCHEMA_POINT_CLOUD2 = b"""std_msgs/Header header
uint32 height
uint32 width
sensor_msgs/PointField[] fields
bool is_bigendian
uint32 point_step
uint32 row_step
uint8[] data
bool is_dense
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: sensor_msgs/PointField
uint8 INT8=1
uint8 UINT8=2
uint8 INT16=3
uint8 UINT16=4
uint8 INT32=5
uint8 UINT32=6
uint8 FLOAT32=7
uint8 FLOAT64=8
string name
uint32 offset
uint8 datatype
uint32 count
"""

#: sensor_msgs/PointField.FLOAT32
PF_FLOAT32 = 7

# ---------------------------------------------------------------------------
# Message builders
# ---------------------------------------------------------------------------


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """Unit quaternion (x, y, z, w) for a rotation of `yaw` about +Z."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def transform_stamped(
    cdr: CdrWriter,
    stamp_ns: int,
    parent: str,
    child: str,
    translation: tuple[float, float, float],
    rotation: tuple[float, float, float, float],
) -> None:
    write_header(cdr, stamp_ns, parent)
    cdr.string(child)
    for component in translation:
        cdr.float64(component)
    for component in rotation:
        cdr.float64(component)


def tf_message(stamp_ns: int, t_seconds: float) -> bytes:
    """world -> base_link (circling + yawing) -> sensor (raised + spinning)."""
    angle = 2.0 * math.pi * t_seconds / 5.0  # one full lap over the 5 s clip
    radius = 2.0
    cdr = CdrWriter()
    cdr.sequence_length(2)
    transform_stamped(
        cdr,
        stamp_ns,
        "world",
        "base_link",
        (radius * math.cos(angle), radius * math.sin(angle), 0.0),
        # Heading along the tangent of the circle, so the triad visibly turns.
        yaw_quaternion(angle + math.pi / 2.0),
    )
    transform_stamped(
        cdr,
        stamp_ns,
        "base_link",
        "sensor",
        (0.0, 0.0, 0.6),
        yaw_quaternion(3.0 * angle),  # spins 3x faster than the base
    )
    return cdr.bytes()


def spiral_shell_points(count: int, phase: float) -> bytes:
    """Packed x/y/z/intensity float32 quadruples on a spherical spiral.

    A Fibonacci-style spiral over a unit sphere scaled to ~1.2 m, rotated by
    `phase` so successive frames differ visibly. Intensity ramps with height so
    an intensity colormap shows a clear gradient.
    """
    raw = bytearray()
    radius = 1.2
    for index in range(count):
        # Latitude sweeps pole to pole; longitude winds many times around it.
        v = (index + 0.5) / count
        polar = math.acos(1.0 - 2.0 * v)
        azimuth = 24.0 * math.pi * v + phase
        x = radius * math.sin(polar) * math.cos(azimuth)
        y = radius * math.sin(polar) * math.sin(azimuth)
        z = radius * math.cos(polar)
        intensity = 0.5 * (z / radius) + 0.5
        raw.extend(struct.pack("<ffff", x, y, z, intensity))
    return bytes(raw)


def point_cloud2(stamp_ns: int, frame_id: str, points: bytes, point_count: int) -> bytes:
    point_step = 16  # 4 x float32
    cdr = CdrWriter()
    write_header(cdr, stamp_ns, frame_id)
    cdr.uint32(1)  # height: 1 => unordered cloud
    cdr.uint32(point_count)  # width
    cdr.sequence_length(4)
    for offset, name in enumerate(("x", "y", "z", "intensity")):
        cdr.string(name)
        cdr.uint32(offset * 4)
        cdr.uint8(PF_FLOAT32)
        cdr.uint32(1)
    cdr.boolean(False)  # is_bigendian
    cdr.uint32(point_step)
    cdr.uint32(point_step * point_count)  # row_step
    cdr.uint8_sequence(points)
    cdr.boolean(True)  # is_dense
    return cdr.bytes()


# ---------------------------------------------------------------------------


def generate(path: Path) -> None:
    duration_s = 5.0
    tf_hz = 20.0
    cloud_hz = 5.0
    point_count = 2000

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as handle:
        writer = Writer(handle)
        writer.start(profile="ros2", library="pj4-scene3d-fixture")

        tf_schema = writer.register_schema(
            name="tf2_msgs/msg/TFMessage", encoding="ros2msg", data=SCHEMA_TF_MESSAGE
        )
        tf_channel = writer.register_channel(
            topic="/tf", message_encoding="cdr", schema_id=tf_schema
        )

        cloud_schema = writer.register_schema(
            name="sensor_msgs/msg/PointCloud2", encoding="ros2msg", data=SCHEMA_POINT_CLOUD2
        )
        cloud_channel = writer.register_channel(
            topic="/points", message_encoding="cdr", schema_id=cloud_schema
        )

        tf_count = int(duration_s * tf_hz)
        for index in range(tf_count):
            t = index / tf_hz
            stamp_ns = int(t * 1e9)
            writer.add_message(
                channel_id=tf_channel,
                log_time=stamp_ns,
                publish_time=stamp_ns,
                data=tf_message(stamp_ns, t),
            )

        cloud_count = int(duration_s * cloud_hz)
        for index in range(cloud_count):
            t = index / cloud_hz
            stamp_ns = int(t * 1e9)
            writer.add_message(
                channel_id=cloud_channel,
                log_time=stamp_ns,
                publish_time=stamp_ns,
                data=point_cloud2(
                    stamp_ns,
                    "sensor",
                    spiral_shell_points(point_count, phase=0.4 * index),
                    point_count,
                ),
            )

        writer.finish()

    print(
        f"[OK] {path} — {path.stat().st_size} bytes, "
        f"{tf_count} /tf msgs, {cloud_count} /points msgs x {point_count} points"
    )


def default_output() -> Path:
    # pj_scene3D/tools/<this file> -> repo root is two levels up.
    return Path(__file__).resolve().parents[2] / "build" / "scene3d_fixture.mcap"


if __name__ == "__main__":
    out = Path(sys.argv[1]).expanduser() if len(sys.argv) > 1 else default_output()
    generate(out.resolve())
