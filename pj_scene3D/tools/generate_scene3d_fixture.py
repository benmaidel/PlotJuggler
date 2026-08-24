#!/usr/bin/env python3
# Copyright 2026 Davide Faconti
# SPDX-License-Identifier: MPL-2.0
"""Generate the synthetic MCAP fixture used by the headless screenshot harnesses.

ONE fixture serves BOTH the 3D harness (``pj_scene3D/tools/screenshot_3d.sh``)
and the 2D harness (``pj_scene2D/tools/screenshot_2d.sh``), so a single run of
this script is enough to exercise either renderer. It is the smallest recording
that covers the data paths those scenes need — a TF tree, a dense point cloud
and a colour image — so a screenshot is a meaningful visual check:

  ``/tf``     ``tf2_msgs/msg/TFMessage``     world -> base_link -> sensor,
              20 Hz for 5 s. ``base_link`` drives a circle around the origin
              while yawing along its heading; ``sensor`` sits above it and
              spins, so the axis triads both translate and rotate.
  ``/points`` ``sensor_msgs/msg/PointCloud2``  a 2000-point spiral shell in the
              ``sensor`` frame, 5 Hz for 5 s. Fields are x/y/z/intensity, all
              FLOAT32 (``PointField.datatype == 7``), ``point_step`` 16.
  ``/markers`` ``visualization_msgs/msg/MarkerArray``  a cube, a sphere and a line
              strip in ``base_link``, 5 Hz for 5 s — three primitive families at
              once, each a different colour and size.
  ``/poses``  ``foxglove_msgs/msg/PosesInFrame``  a 12-pose fan in the ``sensor``
              frame, 10 Hz for 5 s. ``sensor`` circles AND spins, so ignoring the
              frame transform leaves the fan at the origin and ignoring its rotation
              leaves it unspun — two distinct, visible failures.
  ``/map``    ``nav_msgs/msg/OccupancyGrid``   a 40x40 asymmetric costmap at 0.1 m,
              1 Hz for 5 s, published in ``base_link`` — deliberately NOT the fixed
              frame, so a renderer that ignores the fixed_frame<-source_frame
              transform visibly draws it at the world origin instead of under the
              moving robot.
  ``/image``  ``sensor_msgs/msg/Image``       a 320x240 ``rgb8`` test card,
              10 Hz for 5 s. Suppress it with ``--no-image``.

The test card is deliberately ASYMMETRIC — eight vertical colour bars, a thick
top-left-to-bottom-right diagonal, and one filled square in the BOTTOM-LEFT
corner — because a symmetric card cannot tell a correct render from one that is
vertically flipped or has its R and B channels swapped. Every frame carries the
same pixels, so a screenshot is comparable run to run no matter which sample the
tracker happens to sit on.

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
    generate_scene3d_fixture.py [OUTPUT.mcap] [--no-image] [--verify]

``--verify`` re-reads the written file with the independent positional CDR
reader below and asserts the ``/image`` messages round-trip exactly (field
values, ``step == width*3``, and no trailing bytes), which is the only way to
catch a silent CDR alignment mistake — a misaligned field still "renders",
just as garbage.

Default output: ``<repo>/build/scene3d_fixture.mcap`` (a build artifact, never
committed). Requires: ``pip3 install --break-system-packages mcap``.
"""

from __future__ import annotations

import argparse
import math
import struct
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

#: Deliberately the ROS 2 Humble-era definition, with NO ``uv_coordinates`` and no
#: ``mesh_file``. The parser decides whether to expect those optional tail blocks by
#: searching THIS TEXT for those field names, so adding them here without also
#: encoding them would desynchronise every marker after the first.
SCHEMA_MARKER_ARRAY = b"""Marker[] markers
================================================================================
MSG: visualization_msgs/Marker
std_msgs/Header header
string ns
int32 id
int32 type
int32 action
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
std_msgs/ColorRGBA color
builtin_interfaces/Duration lifetime
bool frame_locked
geometry_msgs/Point[] points
std_msgs/ColorRGBA[] colors
string text
string mesh_resource
bool mesh_use_embedded_materials
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: builtin_interfaces/Duration
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a
"""

SCHEMA_POSES_IN_FRAME = b"""builtin_interfaces/Time timestamp
string frame_id
geometry_msgs/Pose[] poses
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
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

SCHEMA_OCCUPANCY_GRID = b"""std_msgs/Header header
MapMetaData info
int8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
================================================================================
MSG: nav_msgs/MapMetaData
builtin_interfaces/Time map_load_time
float32 resolution
uint32 width
uint32 height
geometry_msgs/Pose origin
================================================================================
MSG: geometry_msgs/Pose
Point position
Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
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

SCHEMA_IMAGE = b"""std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

#: sensor_msgs/PointField.FLOAT32
PF_FLOAT32 = 7

#: Geometry of the ``/image`` test card. 320x240 keeps the fixture small while
#: leaving every feature (40 px bars, 48 px corner square) comfortably legible
#: in a screenshot.
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_ENCODING = "rgb8"
#: rgb8 is 3 bytes/pixel and the card is tightly packed, so step == width * 3.
IMAGE_BYTES_PER_PIXEL = 3
#: Header frame_id. Matches the point cloud's frame so a future CameraInfo-based
#: 3D consumer could join them; the 2D viewer ignores it.
IMAGE_FRAME_ID = "sensor"

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


#: Eight vertical bars, left to right. R and B sit at different positions on
#: purpose: a BGR-vs-RGB channel swap moves them, so it cannot hide.
BAR_COLORS = (
    (255, 0, 0),      # red
    (0, 255, 0),      # green
    (0, 0, 255),      # blue
    (255, 255, 255),  # white
    (0, 0, 0),        # black
    (255, 255, 0),    # yellow
    (0, 255, 255),    # cyan
    (255, 0, 255),    # magenta
)

#: Colour of the diagonal and the corner square: absent from BAR_COLORS, so both
#: features stay readable wherever they land.
MARKER_COLOR = (255, 128, 0)  # orange

#: Half-thickness (px) of the diagonal, measured perpendicular-ish along x.
DIAGONAL_HALF_WIDTH = 5
#: Side (px) of the filled square anchored in the BOTTOM-LEFT corner.
CORNER_SQUARE = 48


#: Published in ``base_link`` — moving and yawing, so misplacement is visible.
MARKERS_FRAME_ID = "base_link"
#: visualization_msgs/Marker.type
MARKER_TYPE_CUBE = 1
MARKER_TYPE_SPHERE = 2
MARKER_TYPE_LINE_STRIP = 4


def _write_marker(
    cdr: CdrWriter,
    stamp_ns: int,
    frame_id: str,
    marker_id: int,
    marker_type: int,
    position: tuple[float, float, float],
    scale: tuple[float, float, float],
    color: tuple[float, float, float, float],
    points: list[tuple[float, float, float]],
) -> None:
    """One visualization_msgs/Marker, in the field order the .msg declares.

    Every field is written even when unused (empty strings, zero-length sequences):
    a MarkerArray is a flat sequence, so skipping one leaves every LATER marker
    misaligned rather than failing loudly on this one.
    """
    write_header(cdr, stamp_ns, frame_id)
    cdr.string("fixture")
    cdr.int32(marker_id)
    cdr.int32(marker_type)
    cdr.int32(0)  # action ADD
    for component in position:
        cdr.float64(component)
    for component in (0.0, 0.0, 0.0, 1.0):  # identity orientation
        cdr.float64(component)
    for component in scale:
        cdr.float64(component)
    for component in color:
        cdr.float32(component)
    cdr.int32(0)  # lifetime.sec
    cdr.uint32(0)  # lifetime.nanosec
    cdr.boolean(False)  # frame_locked
    cdr.sequence_length(len(points))
    for point in points:
        for component in point:
            cdr.float64(component)
    cdr.sequence_length(0)  # colors[]
    cdr.string("")  # text
    cdr.string("")  # mesh_resource
    cdr.boolean(False)  # mesh_use_embedded_materials


def marker_array_message(stamp_ns: int, frame_id: str) -> bytes:
    """A cube, a sphere and a line strip — three primitive families at once, each a
    different colour and size so a swapped id or a dropped marker is obvious."""
    cdr = CdrWriter()
    cdr.sequence_length(3)
    _write_marker(cdr, stamp_ns, frame_id, 0, MARKER_TYPE_CUBE, (0.9, 0.0, 0.35),
                  (0.5, 0.5, 0.7), (0.95, 0.35, 0.15, 1.0), [])
    _write_marker(cdr, stamp_ns, frame_id, 1, MARKER_TYPE_SPHERE, (-0.9, 0.0, 0.35),
                  (0.6, 0.4, 0.4), (0.25, 0.55, 0.95, 1.0), [])
    # Offset well clear of the other content: a line is 1 px wide (Metal has no
    # wide-line primitive, and a core-profile GL context rejects glLineWidth > 1), so
    # a strip drawn through the middle of a dense point cloud is invisible even when
    # it renders correctly.
    strip = [(1.7, -1.2 + 0.3 * index, 0.15 + 0.05 * index) for index in range(9)]
    _write_marker(cdr, stamp_ns, frame_id, 2, MARKER_TYPE_LINE_STRIP, (0.0, 0.0, 0.0),
                  (0.04, 0.0, 0.0), (0.15, 0.75, 0.3, 1.0), strip)
    return cdr.bytes()


POSES_COUNT = 12
#: Published in ``sensor``, which both circles the origin and SPINS three times
#: faster than the base — so a renderer that ignores the frame transform leaves the
#: fan at the world origin, and one that ignores its ROTATION leaves it unspun. Both
#: failures are visible; neither is if everything is published in the fixed frame.
POSES_FRAME_ID = "sensor"


def poses_in_frame_message(stamp_ns: int, frame_id: str, count: int) -> bytes:
    """foxglove_msgs/msg/PosesInFrame.

    NOTE the first field is a BARE builtin_interfaces/Time, not a std_msgs/Header —
    writing a Header here would shift every subsequent field.

    The poses are a fan at increasing radius with yaw tangent to it, which makes both
    the placement and the orientation of each arm checkable by eye.
    """
    cdr = CdrWriter()
    cdr.int32(stamp_ns // 1_000_000_000)
    cdr.uint32(stamp_ns % 1_000_000_000)
    cdr.string(frame_id)
    cdr.sequence_length(count)
    for index in range(count):
        fraction = index / max(count - 1, 1)
        angle = fraction * 1.6
        radius = 0.4 + 0.9 * fraction
        for component in (radius * math.cos(angle), radius * math.sin(angle), 0.25):
            cdr.float64(component)
        # Yaw tangent to the fan: q = (0, 0, sin(yaw/2), cos(yaw/2)).
        yaw = angle + math.pi / 2.0
        for component in (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)):
            cdr.float64(component)
    return cdr.bytes()


OCCUPANCY_WIDTH = 40
OCCUPANCY_HEIGHT = 40
OCCUPANCY_RESOLUTION = 0.1
#: The map is published in ``base_link``, NOT the fixed frame. That is deliberate:
#: base_link circles the origin while yawing, so a renderer that ignores the
#: fixed_frame<-source_frame transform draws the map at the world origin instead of
#: under the moving robot — a placement bug that is invisible when every topic
#: happens to be published in the fixed frame.
OCCUPANCY_FRAME_ID = "base_link"


def occupancy_cells(width: int, height: int) -> bytes:
    """An ASYMMETRIC costmap: free interior, an occupied L along two edges, a lethal
    blob off-centre, and an unknown (-1) border.

    Asymmetry is the point, exactly as for the image test card: a symmetric map
    cannot distinguish a correct render from one mirrored or transposed.
    """
    cells = bytearray(width * height)
    for row in range(height):
        for col in range(width):
            value = 0  # free
            if row < 2 or col < 2 or row >= height - 2 or col >= width - 2:
                value = -1  # unknown border
            elif row == 8 and 6 <= col < width - 10:
                value = 100  # long occupied wall
            elif col == 6 and 8 <= row < height - 12:
                value = 100  # the L's short leg
            else:
                dx = col - 27
                dy = row - 26
                if dx * dx + dy * dy < 16:
                    value = 100  # lethal blob, off-centre
            cells[row * width + col] = value & 0xFF
    return bytes(cells)


def occupancy_grid_message(stamp_ns: int, frame_id: str, width: int, height: int, cells: bytes) -> bytes:
    """nav_msgs/msg/OccupancyGrid. Field order per the .msg: header, info, data.

    The origin places the map's lower-front-left CORNER, so it is offset by half the
    map extent to centre the map on the frame it is published in.
    """
    cdr = CdrWriter()
    write_header(cdr, stamp_ns, frame_id)
    # info.map_load_time — the grid uses the header stamp, so zero is fine.
    cdr.int32(0)
    cdr.uint32(0)
    cdr.float32(OCCUPANCY_RESOLUTION)
    cdr.uint32(width)
    cdr.uint32(height)
    half_w = 0.5 * width * OCCUPANCY_RESOLUTION
    half_h = 0.5 * height * OCCUPANCY_RESOLUTION
    for component in (-half_w, -half_h, 0.0):
        cdr.float64(component)
    for component in (0.0, 0.0, 0.0, 1.0):  # identity orientation
        cdr.float64(component)
    cdr.uint8_sequence(cells)
    return cdr.bytes()


def image_test_card(width: int = IMAGE_WIDTH, height: int = IMAGE_HEIGHT) -> bytes:
    """Tightly packed ``rgb8`` bytes, row 0 first (ROS raster order: top row first).

    Deliberately asymmetric in both axes so a screenshot distinguishes a correct
    render from a flipped, mirrored or channel-swapped one:

      * eight vertical colour bars (red at x=0 ... magenta at x=width)
      * a thick diagonal from the TOP-LEFT to the BOTTOM-RIGHT corner
      * a filled square in the BOTTOM-LEFT corner only

    A vertical flip turns the diagonal into a "/" and lifts the square to the
    top-left; a horizontal mirror moves the red bar to the right edge; an R/B
    swap turns the leftmost bar blue. None of those survive inspection.
    """
    bar_width = width / len(BAR_COLORS)
    raw = bytearray()
    for y in range(height):
        # x of the diagonal on this row: 0 at the top row, width-1 at the bottom.
        diagonal_x = y * (width - 1) / (height - 1)
        in_square_rows = y >= height - CORNER_SQUARE
        for x in range(width):
            if in_square_rows and x < CORNER_SQUARE:
                color = MARKER_COLOR
            elif abs(x - diagonal_x) <= DIAGONAL_HALF_WIDTH:
                color = MARKER_COLOR
            else:
                color = BAR_COLORS[min(int(x / bar_width), len(BAR_COLORS) - 1)]
            raw.extend(color)
    return bytes(raw)


def image_message(stamp_ns: int, frame_id: str, width: int, height: int, pixels: bytes) -> bytes:
    """sensor_msgs/msg/Image. Field order per the .msg: header, height, width,
    encoding, is_bigendian, step, data."""
    cdr = CdrWriter()
    write_header(cdr, stamp_ns, frame_id)
    cdr.uint32(height)
    cdr.uint32(width)
    cdr.string(IMAGE_ENCODING)
    cdr.uint8(0)  # is_bigendian: the pattern is byte-per-channel, so moot
    cdr.uint32(width * IMAGE_BYTES_PER_PIXEL)  # step
    cdr.uint8_sequence(pixels)
    return cdr.bytes()


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
# Verification
# ---------------------------------------------------------------------------


class CdrReader:
    """Positional little-endian CDR decoder, written independently of CdrWriter.

    Deliberately NOT sharing code with the writer: a reader that mirrors the
    writer's own alignment bug agrees with it and proves nothing. This one
    tracks an absolute cursor and derives alignment from the payload start (the
    byte after the 4-byte encapsulation header), which is the rule the spec
    states — so a writer that forgot a pad shows up here as a wrong value or as
    leftover trailing bytes.
    """

    def __init__(self, raw: bytes) -> None:
        if len(raw) < 4:
            raise ValueError("CDR payload shorter than its encapsulation header")
        self._raw = raw
        self._base = 4  # payload start == alignment origin
        self._pos = 4

    def _align(self, size: int) -> None:
        pad = (self._pos - self._base) % size
        if pad:
            self._pos += size - pad

    def uint8(self) -> int:
        value = self._raw[self._pos]
        self._pos += 1
        return value

    def int32(self) -> int:
        self._align(4)
        (value,) = struct.unpack_from("<i", self._raw, self._pos)
        self._pos += 4
        return value

    def uint32(self) -> int:
        self._align(4)
        (value,) = struct.unpack_from("<I", self._raw, self._pos)
        self._pos += 4
        return value

    def string(self) -> str:
        length = self.uint32()  # INCLUDES the NUL
        raw = self._raw[self._pos : self._pos + length]
        self._pos += length
        if not raw.endswith(b"\x00"):
            raise ValueError("CDR string not NUL-terminated")
        return raw[:-1].decode("utf-8")

    def uint8_sequence(self) -> bytes:
        count = self.uint32()
        raw = self._raw[self._pos : self._pos + count]
        if len(raw) != count:
            raise ValueError(f"sequence truncated: wanted {count}, have {len(raw)}")
        self._pos += count
        return raw

    def remaining(self) -> int:
        return len(self._raw) - self._pos


def verify_image_messages(path: Path, expected_pixels: bytes) -> int:
    """Re-read every ``/image`` message and assert it decodes byte-exactly.

    Returns the number of messages checked; raises AssertionError on the first
    mismatch. Cheap enough to run on every generation.
    """
    from mcap.reader import make_reader

    checked = 0
    with path.open("rb") as handle:
        for schema, channel, message in make_reader(handle).iter_messages(topics=["/image"]):
            assert schema is not None and schema.name == "sensor_msgs/msg/Image", schema
            assert schema.encoding == "ros2msg", schema.encoding
            assert channel.message_encoding == "cdr", channel.message_encoding

            reader = CdrReader(message.data)
            sec = reader.int32()
            nanosec = reader.uint32()
            frame_id = reader.string()
            height = reader.uint32()
            width = reader.uint32()
            encoding = reader.string()
            is_bigendian = reader.uint8()
            step = reader.uint32()
            data = reader.uint8_sequence()

            stamp_ns = sec * 1_000_000_000 + nanosec
            assert stamp_ns == message.log_time, (stamp_ns, message.log_time)
            assert frame_id == IMAGE_FRAME_ID, frame_id
            assert (width, height) == (IMAGE_WIDTH, IMAGE_HEIGHT), (width, height)
            assert encoding == IMAGE_ENCODING, encoding
            assert is_bigendian == 0, is_bigendian
            assert step == width * IMAGE_BYTES_PER_PIXEL, (step, width)
            assert len(data) == step * height, (len(data), step * height)
            assert data == expected_pixels, "pixel bytes did not round-trip"
            # The tell for a bogus alignment pad: the reader must land exactly on
            # the end of the payload, having consumed every byte the writer wrote.
            assert reader.remaining() == 0, f"{reader.remaining()} trailing byte(s)"
            checked += 1

    assert checked > 0, "no /image messages found to verify"
    return checked


# ---------------------------------------------------------------------------


def generate(path: Path, include_image: bool = True) -> None:
    duration_s = 5.0
    tf_hz = 20.0
    cloud_hz = 5.0
    image_hz = 10.0
    poses_hz = 10.0
    markers_hz = 5.0
    # The map is static content published slowly, as a real map server does.
    occupancy_hz = 1.0
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

        markers_schema = writer.register_schema(
            name="visualization_msgs/msg/MarkerArray", encoding="ros2msg", data=SCHEMA_MARKER_ARRAY
        )
        markers_channel = writer.register_channel(
            topic="/markers", message_encoding="cdr", schema_id=markers_schema
        )

        poses_schema = writer.register_schema(
            name="foxglove_msgs/msg/PosesInFrame", encoding="ros2msg", data=SCHEMA_POSES_IN_FRAME
        )
        poses_channel = writer.register_channel(
            topic="/poses", message_encoding="cdr", schema_id=poses_schema
        )

        occupancy_schema = writer.register_schema(
            name="nav_msgs/msg/OccupancyGrid", encoding="ros2msg", data=SCHEMA_OCCUPANCY_GRID
        )
        occupancy_channel = writer.register_channel(
            topic="/map", message_encoding="cdr", schema_id=occupancy_schema
        )

        image_channel = None
        if include_image:
            image_schema = writer.register_schema(
                name="sensor_msgs/msg/Image", encoding="ros2msg", data=SCHEMA_IMAGE
            )
            image_channel = writer.register_channel(
                topic="/image", message_encoding="cdr", schema_id=image_schema
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

        markers_count = int(duration_s * markers_hz)
        for index in range(markers_count):
            stamp_ns = int(index / markers_hz * 1e9)
            writer.add_message(
                channel_id=markers_channel,
                log_time=stamp_ns,
                publish_time=stamp_ns,
                data=marker_array_message(stamp_ns, MARKERS_FRAME_ID),
            )

        poses_count = int(duration_s * poses_hz)
        for index in range(poses_count):
            stamp_ns = int(index / poses_hz * 1e9)
            writer.add_message(
                channel_id=poses_channel,
                log_time=stamp_ns,
                publish_time=stamp_ns,
                data=poses_in_frame_message(stamp_ns, POSES_FRAME_ID, POSES_COUNT),
            )

        # One shared payload: the map is static, and what it exercises is PLACEMENT in
        # a non-fixed frame, which the moving base_link already varies for us.
        occupancy_payload = occupancy_cells(OCCUPANCY_WIDTH, OCCUPANCY_HEIGHT)
        occupancy_count = int(duration_s * occupancy_hz)
        for index in range(occupancy_count):
            stamp_ns = int(index / occupancy_hz * 1e9)
            writer.add_message(
                channel_id=occupancy_channel,
                log_time=stamp_ns,
                publish_time=stamp_ns,
                data=occupancy_grid_message(
                    stamp_ns,
                    OCCUPANCY_FRAME_ID,
                    OCCUPANCY_WIDTH,
                    OCCUPANCY_HEIGHT,
                    occupancy_payload,
                ),
            )

        image_count = 0
        if image_channel is not None:
            # One shared payload for every frame: the pattern is time-invariant by
            # design (see the module docstring), and reusing the bytes keeps both
            # the generation cost and the zstd-chunked file size down.
            pixels = image_test_card()
            image_count = int(duration_s * image_hz)
            for index in range(image_count):
                stamp_ns = int(index / image_hz * 1e9)
                writer.add_message(
                    channel_id=image_channel,
                    log_time=stamp_ns,
                    publish_time=stamp_ns,
                    data=image_message(stamp_ns, IMAGE_FRAME_ID, IMAGE_WIDTH, IMAGE_HEIGHT, pixels),
                )

        writer.finish()

    summary = f"{tf_count} /tf msgs, {cloud_count} /points msgs x {point_count} points"
    summary += f", {markers_count} /markers msgs x3 in {MARKERS_FRAME_ID}"
    summary += f", {poses_count} /poses msgs x {POSES_COUNT} in {POSES_FRAME_ID}"
    summary += f", {occupancy_count} /map msgs {OCCUPANCY_WIDTH}x{OCCUPANCY_HEIGHT} in {OCCUPANCY_FRAME_ID}"
    if image_count:
        summary += f", {image_count} /image msgs {IMAGE_WIDTH}x{IMAGE_HEIGHT} {IMAGE_ENCODING}"
    print(f"[OK] {path} — {path.stat().st_size} bytes, {summary}")


def default_output() -> Path:
    # pj_scene3D/tools/<this file> -> repo root is two levels up.
    return Path(__file__).resolve().parents[2] / "build" / "scene3d_fixture.mcap"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "output", nargs="?", type=Path, default=None, help="destination .mcap (default: build/scene3d_fixture.mcap)"
    )
    parser.add_argument(
        "--no-image",
        dest="image",
        action="store_false",
        help="omit the /image test card (the 2D harness needs it; the 3D one does not)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="re-read the /image messages with the independent CDR reader and assert they round-trip",
    )
    args = parser.parse_args()

    out = (args.output.expanduser() if args.output is not None else default_output()).resolve()
    generate(out, include_image=args.image)
    if args.verify:
        if not args.image:
            parser.error("--verify has nothing to check with --no-image")
        checked = verify_image_messages(out, image_test_card())
        print(f"[OK] verified {checked} /image message(s): CDR round-trips with zero trailing bytes")


if __name__ == "__main__":
    main()
