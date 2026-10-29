#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Unit tests of the RGB LiDAR pseudo sensor (no CARLA server needed).
Run with the workspace sourced: python3 -m pytest carla_ros_bridge/test/test_rgb_lidar.py
"""

import math
import os
import sys
from threading import Lock
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

import carla
from sensor_msgs.msg import PointField
from std_msgs.msg import Header

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from carla_ros_bridge.rgb_lidar_sensor import (  # noqa: E402
    FIELDS, LIDAR_ROLE, POINT_DTYPE, RgbLidarSensor, colorize, slice_fields)

CAM_ROLE = "rgb_lidar_cam_front"
DT = 0.01


def make_image(transform=None, width=4, height=2, fov=90.0):
    """carla.Image stand-in whose pixel (v, u) is BGRA (u, v, 10 * u + v, 255)."""
    v, u = np.mgrid[0:height, 0:width]
    bgra = np.stack([u, v, 10 * u + v, np.full_like(u, 255)], axis=-1).astype(np.uint8)
    return SimpleNamespace(transform=transform or carla.Transform(), width=width, height=height,
                           fov=fov, raw_data=bgra.tobytes())


def rgb_of(u, v):
    """Packed 0x00RRGGBB of pixel (v, u) of make_image: the alpha byte (255) must not leak in."""
    return (10 * u + v) << 16 | v << 8 | u


# --- colorize ------------------------------------------------------------------------------------

def test_colorize_optical_axis_bgra_to_rgb():
    # w=4, fov=90: f = 2, the optical axis hits pixel (u=2, v=1)
    colors = colorize(np.array([[10.0, 0.0, 0.0]]), np.eye(4), [make_image()])
    assert colors.dtype == np.uint32
    assert colors.tolist() == [rgb_of(2, 1)]


def test_colorize_uncolored_is_zero():
    behind, outside = [-10.0, 0.0, 0.0], [1.0, 5.0, 0.0]
    colors = colorize(np.array([behind, outside]), np.eye(4), [make_image()])
    assert colors.tolist() == [0, 0]


def test_colorize_closest_optical_axis_wins():
    front = make_image()
    back = make_image(carla.Transform(carla.Location(), carla.Rotation(yaw=180.0)), width=6)
    # off the pixel borders: the yaw 180 matrix is not exact (f = 3 for the back camera)
    colors = colorize(np.array([[10.0, 0.0, 0.0], [-10.0, -1.0, -0.5]]), np.eye(4), [front, back])
    assert colors.tolist() == [rgb_of(2, 1), rgb_of(3, 1)]


def test_colorize_image_border_is_clamped():
    # y / x = 1 projects at u = w, on the image border
    colors = colorize(np.array([[10.0, 10.0, 0.0]]), np.eye(4), [make_image()])
    assert colors.tolist() == [rgb_of(3, 1)]


def test_colorize_uses_lidar_pose():
    lidar_to_world = np.array(carla.Transform(carla.Location(), carla.Rotation(yaw=180.0)).get_matrix())
    # (10, -1, -0.5) in the world: u = floor(2 - 2 * 0.1), v = floor(1 + 2 * 0.05)
    colors = colorize(np.array([[-10.0, 1.0, -0.5]]), lidar_to_world, [make_image()])
    assert colors.tolist() == [rgb_of(1, 1)]


# --- layout --------------------------------------------------------------------------------------

def test_fields_match_ouster_color_point():
    # ouster_ros::ColorPoint through pcl::toROSMsg
    expected = [('x', 0, PointField.FLOAT32), ('y', 4, PointField.FLOAT32), ('z', 8, PointField.FLOAT32),
                ('intensity', 16, PointField.FLOAT32), ('t', 20, PointField.UINT32),
                ('reflectivity', 24, PointField.UINT8), ('ring', 26, PointField.UINT16),
                ('ambient', 28, PointField.UINT16), ('range', 32, PointField.UINT32),
                ('rgb', 36, PointField.UINT32)]
    assert [(f.name, f.offset, f.datatype) for f in FIELDS] == expected
    assert all(f.count == 1 for f in FIELDS)
    assert POINT_DTYPE.itemsize == 48


# --- slice_fields --------------------------------------------------------------------------------

def points_at(azimuths_deg, distance=10.0, elevation_deg=0.0):
    a, e = np.radians(azimuths_deg), math.radians(elevation_deg)
    return np.c_[distance * np.cos(e) * np.cos(a), distance * np.cos(e) * np.sin(a),
                 np.full(len(a), distance * math.sin(e))]


def end_angle(start_yaw_deg):
    """horizontal_angle of a measurement whose rays start at start_yaw_deg (10 slices per turn)."""
    return math.radians(math.fmod(start_yaw_deg + 180.0 + 36.0, 360.0))


@pytest.mark.parametrize("start_yaw, azimuths, fractions", [
    (108.0, [108.0, 125.0, 143.9], [0.0, 17 / 36, 35.9 / 36]),
    (-18.0, [-18.0, 0.0, 17.9], [0.0, 0.5, 35.9 / 36]),
    (162.0, [162.0, 179.9, -179.9, -162.1], [0.0, 17.9 / 36, 18.1 / 36, 35.9 / 36]),  # across +-180
])
def test_slice_fields_t_is_slice_offset_plus_azimuth_fraction(start_yaw, azimuths, fractions):
    index = 3
    points = points_at(azimuths).astype(np.float32).astype(np.float64)
    _, t, _ = slice_fields(points, [len(points)], index, end_angle(start_yaw), 10, DT)
    np.testing.assert_allclose(t, (index + np.array(fractions)) * DT * 1e9, atol=1e3)
    assert t.dtype == np.uint32
    assert np.all(np.diff(t.astype(np.int64)) > 0)
    assert np.all((t >= index * DT * 1e9) & (t < (index + 1) * DT * 1e9))


def test_slice_fields_ring_and_range():
    points = np.r_[points_at([0.0, 5.0], 12.3456, 10.0), points_at([1.0], 7.0, 0.0),
                   points_at([2.0, 3.0, 4.0], 3.21, -10.0)]
    ring, _, distance = slice_fields(points, [2, 1, 3], 0, end_angle(-18.0), 10, DT)
    assert ring.dtype == np.uint16 and ring.tolist() == [0, 0, 1, 2, 2, 2]
    assert distance.dtype == np.uint32 and distance.tolist() == [12346, 12346, 7000, 3210, 3210, 3210]


# --- turn assembly -------------------------------------------------------------------------------

def make_sensor(ticks_per_scan=3):
    sensor = RgbLidarSensor.__new__(RgbLidarSensor)
    sensor.name = "rgb_lidar"
    sensor.ticks_per_scan = ticks_per_scan
    sensor._dt = DT
    sensor._horizontal_fov = 360.0
    sensor.cameras = [SimpleNamespace(attributes={"role_name": CAM_ROLE})]
    sensor._lock = Lock()
    sensor._frames = {}
    sensor._overflow_drops = 0
    sensor._first_frame = None
    sensor._scan = []
    sensor._scan_id = None
    sensor.node = mock.Mock(parameters={"world_frame": "carla_world"})
    sensor.publisher = mock.Mock()
    sensor._tf_broadcaster = mock.Mock()
    sensor.get_prefix = lambda: "hero/rgb_lidar"
    sensor.get_msg_header = mock.Mock(side_effect=lambda frame_id=None, timestamp=None: Header(
        frame_id=frame_id or "hero/rgb_lidar"))
    return sensor


def make_frame(frame, ticks_per_scan=3):
    """Slice of 2 points (one per channel) at frame, a turn starting at frame 100."""
    index = (frame - 100) % ticks_per_scan
    arc = 360.0 / ticks_per_scan
    start_yaw = index * arc - 180.0
    points = points_at([start_yaw + arc / 4, start_yaw + arc / 2])
    raw = np.c_[points, [0.5, 0.25]].astype(np.float32)
    lidar = SimpleNamespace(frame=frame, timestamp=frame * DT, raw_data=raw.tobytes(),
                            transform=carla.Transform(), channels=2, get_point_count=lambda ch: 1,
                            horizontal_angle=math.radians(math.fmod((index + 1) * arc, 360.0)))
    up = carla.Transform(carla.Location(), carla.Rotation(pitch=90.0))  # sees no point of the horizon
    return {LIDAR_ROLE: lidar, CAM_ROLE: make_image(up)}


def feed(sensor, frame):
    """Delivers the frame's data through the sensor callbacks, LiDAR first."""
    for role in (LIDAR_ROLE, CAM_ROLE):
        sensor._on_data(role, SimpleNamespace(frame=frame[LIDAR_ROLE].frame, **vars(frame[role]))
                        if role == CAM_ROLE else frame[role])


def published(sensor):
    """(stamp time, structured cloud) of each published turn."""
    stamps = [c.kwargs["timestamp"] for c in sensor.get_msg_header.call_args_list
              if c.kwargs.get("frame_id") is None]
    clouds = [np.frombuffer(c.args[0].data, POINT_DTYPE) for c in sensor.publisher.publish.call_args_list]
    return list(zip(stamps, clouds))


def test_turn_published_once_at_its_first_slice():
    sensor = make_sensor()
    for frame in (100, 101, 102):
        feed(sensor, make_frame(frame))
    [(stamp, cloud)] = published(sensor)
    assert stamp == pytest.approx(100 * DT)
    assert sensor._tf_broadcaster.sendTransform.call_count == 3
    msg = sensor.publisher.publish.call_args.args[0]
    assert msg.point_step == 48 and msg.width == 6 and len(msg.data) == 6 * 48
    assert cloud['ring'].tolist() == [0, 1] * 3
    # a quarter and a half of each 120 deg slice
    np.testing.assert_allclose(cloud['t'], [(i + f) * DT * 1e9 for i in range(3) for f in (0.25, 0.5)], atol=1e3)
    assert cloud['range'].tolist() == [10000] * 6
    assert np.all(cloud['reflectivity'] == 0) and np.all(cloud['ambient'] == 0)
    assert np.all(cloud['rgb'] == 0)  # the camera sees none of these points
    np.testing.assert_allclose(cloud['intensity'], [0.5, 0.25] * 3)
    sensor.node.logwarn.assert_not_called()


def test_rgb_is_packed_uint32():
    sensor = make_sensor(ticks_per_scan=1)
    frame = make_frame(100, ticks_per_scan=1)
    frame[CAM_ROLE] = make_image()
    frame[LIDAR_ROLE].raw_data = np.array([[10.0, 0.0, 0.0, 1.0]], np.float32).tobytes()
    frame[LIDAR_ROLE].get_point_count = lambda ch: [1, 0][ch]
    feed(sensor, frame)
    [(_, cloud)] = published(sensor)
    assert cloud['rgb'].tolist() == [rgb_of(2, 1)]
    assert cloud['y'].tolist() == [0.0]


def test_dropped_first_slice_keeps_turn_timing_and_warns():
    sensor = make_sensor()
    sensor._first_frame = 100
    for frame in (104, 105):  # frame 103 of the turn 103-105 was dropped
        feed(sensor, make_frame(frame))
    [(stamp, cloud)] = published(sensor)
    assert stamp == pytest.approx(103 * DT)
    assert cloud['t'].min() >= DT * 1e9
    sensor.node.logwarn.assert_called_once()
    assert "2/3" in sensor.node.logwarn.call_args.args[0]


def test_dropped_last_slice_publishes_leftovers_and_warns():
    sensor = make_sensor()
    for frame in (100, 101, 103, 104, 105):  # frame 102 was dropped
        feed(sensor, make_frame(frame))
    (stamp0, cloud0), (stamp1, cloud1) = published(sensor)
    assert stamp0 == pytest.approx(100 * DT) and len(cloud0) == 4
    assert stamp1 == pytest.approx(103 * DT) and len(cloud1) == 6
    sensor.node.logwarn.assert_called_once()

