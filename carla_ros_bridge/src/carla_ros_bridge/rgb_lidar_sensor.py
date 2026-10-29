#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle an RGB LiDAR pseudo sensor
"""

import math
from threading import Lock

import numpy as np
import tf2_ros

import carla_common.transforms as trans
from carla_ros_bridge.pseudo_actor import PseudoActor

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField

LIDAR_ROLE = "rgb_lidar"
CAMERA_ROLE_PREFIX = "rgb_lidar_cam"
UNCOLORED = 0  # packed rgb of the points no camera sees, as ouster-ros, which leaves rgb = 0 without color
# incomplete frames kept while the rest of the rig arrives (camera data lags the LiDAR by ~2 ticks):
# a rig whose sensors never capture on the same frame would otherwise buffer images without bound
MAX_PENDING_FRAMES = 8
AZIMUTH_EPS_DEG = 0.01  # tolerance on the first ray of a slice, recovered from float32 points

# ouster_ros::ColorPoint (ouster-ros color_point.h, point_type color_point) as laid out by
# pcl::toROSMsg: PCL_ADD_POINT4D pads z to 16 bytes, EIGEN_ALIGN16 pads the point to 48
POINT_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 't', 'reflectivity', 'ring', 'ambient', 'range', 'rgb'],
    'formats': ['<f4', '<f4', '<f4', '<f4', '<u4', 'u1', '<u2', '<u2', '<u4', '<u4'],
    'offsets': [0, 4, 8, 16, 20, 24, 26, 28, 32, 36],
    'itemsize': 48,
})
_DATATYPES = {'<f4': PointField.FLOAT32, '<u4': PointField.UINT32, '|u1': PointField.UINT8,
              '<u2': PointField.UINT16}
FIELDS = [PointField(name=name, offset=POINT_DTYPE.fields[name][1],
                     datatype=_DATATYPES[POINT_DTYPE.fields[name][0].str], count=1)
          for name in POINT_DTYPE.names]


def colorize(points, lidar_to_world, images):
    """
    Colors each LiDAR point (Nx3, CARLA frame) with the pixel of the camera whose
    optical axis is closest to it. images: carla.Image of the same frame.
    Returns N uint32 packed 0x00RRGGBB, UNCOLORED where no camera sees the point.
    """
    # camera <- LiDAR composed in float64 (world coordinates can be large), applied in float32 to the
    # points as 3xN rows for all the cameras at once: cam[i] holds x, y, z in the frame of camera i
    cam_from_lidar = np.stack([np.linalg.inv(np.array(img.transform.get_matrix())) @ lidar_to_world
                               for img in images])[:, :3].astype(np.float32)
    cam = cam_from_lidar[:, :, :3] @ np.ascontiguousarray(points, np.float32).T + cam_from_lidar[:, :, 3:]
    x = cam[:, 0]
    cosines = x / np.maximum(np.sqrt(np.einsum('cin,cin->cn', cam, cam)), 1e-6)
    best = np.argmax(cosines, axis=0)

    colors = np.full(cam.shape[2], UNCOLORED, np.uint32)
    for i, img in enumerate(images):
        w, h = img.width, img.height
        f = w / (2.0 * math.tan(math.radians(img.fov) / 2.0))
        idx = np.flatnonzero((best == i) & (x[i] > 0))
        px, py, pz = cam[i][:, idx]
        u = w / 2.0 + f * py / px
        v = h / 2.0 - f * pz / px
        inside = (u >= 0) & (u < w + 1) & (v >= 0) & (v < h + 1)  # floor in [0, size]: image border included
        u = np.minimum(u[inside].astype(np.intp), w - 1)  # truncation is floor, u >= 0
        v = np.minimum(v[inside].astype(np.intp), h - 1)
        pixels = np.frombuffer(img.raw_data, '<u4')  # BGRA bytes: 0xAARRGGBB
        colors[idx[inside]] = pixels[v * w + u] & 0xFFFFFF
    return colors


def slice_fields(points, channel_counts, index, horizontal_angle, ticks_per_scan, dt, horizontal_fov=360.0):
    """
    Ouster fields of one LiDAR slice (points Nx3, CARLA frame, grouped by channel as CARLA sends them).
    horizontal_angle: of the measurement (rad), the rotation at the end of its tick.
    Returns (ring, t, range): ring = channel index (0 = upper_fov, as Ouster's top beam),
    t = ns since the start of the turn (slice offset + azimuth fraction of the slice sweep),
    range = distance in mm.
    """
    ring = np.repeat(np.arange(len(channel_counts), dtype=np.uint16), channel_counts)
    # CARLA casts the rays of a tick at yaw fmod(start + k * step, fov) - fov / 2 over arc = fov / ticks_per_scan
    arc = horizontal_fov / ticks_per_scan
    start = math.fmod(math.degrees(horizontal_angle) - arc, horizontal_fov) - horizontal_fov / 2.0
    azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    swept = np.mod(azimuth - start + AZIMUTH_EPS_DEG, horizontal_fov) - AZIMUTH_EPS_DEG
    fraction = np.clip(swept / arc, 0.0, 1.0)
    t = np.round((index + fraction) * dt * 1e9).astype(np.uint32)
    distance = np.round(np.sqrt(np.einsum('ij,ij->i', points, points)) * 1000.0).astype(np.uint32)
    return ring, t, distance


class RgbLidarSensor(PseudoActor):

    """
    Pseudo sensor fusing the parent's LiDAR (role_name LIDAR_ROLE) with its RGB
    cameras (role_name CAMERA_ROLE_PREFIX*): each LiDAR slice is colored with the
    images of the same frame, and every turn is published as a PointCloud2 on
    /carla/<parent>/<name> with the fields of ouster-ros color_point (POINT_DTYPE),
    stamped at the start of its first slice; t is the time since the stamp. The
    points of each slice stay in the LiDAR frame of that slice, as with a real
    rotating LiDAR, and TF world_frame -> LiDAR frame is sent at every slice.
    Turns missing slices are published anyway, as ouster-ros does, with a warning.
    With sensor_tick > 0 the rig must share it and the LiDAR must do a whole turn
    per tick: each capture is then a full turn.
    """

    def __init__(self, uid, name, parent, node):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        """
        super(RgbLidarSensor, self).__init__(uid=uid,
                                             name=name,
                                             parent=parent,
                                             node=node)
        world = node.carla_world
        rig = [a for a in world.get_actors().filter("sensor.*")
               if a.parent is not None and a.parent.id == parent.carla_actor.id]
        lidars = [a for a in rig if a.type_id == "sensor.lidar.ray_cast"
                  and a.attributes.get("role_name") == LIDAR_ROLE]
        self.cameras = [a for a in rig if a.type_id == "sensor.camera.rgb"
                        and a.attributes.get("role_name", "").startswith(CAMERA_ROLE_PREFIX)]
        if not lidars or not self.cameras:
            raise RuntimeError("{}: no '{}' LiDAR or '{}*' cameras on parent {}".format(
                name, LIDAR_ROLE, CAMERA_ROLE_PREFIX, parent.carla_actor.id))
        self.lidar = lidars[0]

        dt = self._dt = world.get_settings().fixed_delta_seconds
        self._horizontal_fov = float(self.lidar.attributes.get("horizontal_fov", 360.0))
        if not dt:
            raise RuntimeError("{}: needs a fixed_delta_seconds".format(name))
        ticks = 1.0 / (float(self.lidar.attributes["rotation_frequency"]) * dt)
        self.ticks_per_scan = round(ticks)
        if self.ticks_per_scan < 1 or abs(ticks - self.ticks_per_scan) > 1e-6:
            raise RuntimeError("{}: 1 / (rotation_frequency * fixed_delta_seconds) = {:.3f} must be a positive "
                               "integer".format(name, ticks))
        sensor_ticks = {float(a.attributes["sensor_tick"]) for a in [self.lidar] + self.cameras}
        if len(sensor_ticks) > 1:
            raise RuntimeError("{}: the rig sensors must share sensor_tick, got {}".format(name, sorted(sensor_ticks)))
        if sensor_ticks != {0.0} and self.ticks_per_scan != 1:
            raise RuntimeError("{}: with sensor_tick > 0 the LiDAR must turn once per tick "
                               "(rotation_frequency = 1 / fixed_delta_seconds)".format(name))

        self._lock = Lock()
        self._frames = {}       # frame -> {role_name: carla sensor data}
        self._overflow_drops = 0  # incomplete frames dropped by MAX_PENDING_FRAMES
        self._first_frame = None
        self._scan = []         # (index, timestamp, raw, rgb, ring, t, range) of the slices of the current turn
        self._scan_id = None

        self.publisher = node.new_publisher(PointCloud2, self.get_topic_prefix(), qos_profile=10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(node)
        for actor in [self.lidar] + self.cameras:
            role = actor.attributes["role_name"]
            actor.listen(lambda data, role=role: self._on_data(role, data))

    def destroy(self):
        """
        Function (override) to destroy this object.
        Stops listening to the rig sensors still alive (stop() on a destroyed one raises).
        """
        for actor in [self.lidar] + self.cameras:
            try:
                actor.stop()
            except RuntimeError:  # already destroyed by its owner, is_alive may still be True
                pass
        self.node.destroy_publisher(self.publisher)
        super(RgbLidarSensor, self).destroy()

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.rgb_lidar"

    def _on_data(self, role, data):
        """Buffers the data by frame; a frame with the LiDAR and every camera is fused, older incomplete ones dropped."""
        with self._lock:
            frame = self._frames.setdefault(data.frame, {})
            frame[role] = data
            if len(frame) < 1 + len(self.cameras):
                if len(self._frames) > MAX_PENDING_FRAMES:
                    del self._frames[min(self._frames)]
                    self._overflow_drops += 1
                    if self._overflow_drops % 100 == 1:
                        self.node.logwarn("{}: frames are not completed by the LiDAR and every camera, {} dropped: "
                                          "do the rig sensors capture on the same frames?".format(
                                              self.name, self._overflow_drops))
                return
            stale = [f for f in self._frames if f < data.frame]
            if stale:
                self.node.logwarn("{}: dropped {} incomplete frames".format(self.name, len(stale)))
            for f in stale + [data.frame]:
                del self._frames[f]
            self._add_slice(frame)

    def _add_slice(self, frame):
        """Colors the frame's LiDAR slice, sends its TF and publishes the turn at its last slice."""
        lidar = frame[LIDAR_ROLE]
        if self._first_frame is None:
            self._first_frame = lidar.frame
        scan_id, index = divmod(lidar.frame - self._first_frame, self.ticks_per_scan)
        if scan_id != self._scan_id:
            if self._scan:  # a turn whose last slice was dropped
                self._publish(self._scan_id)
            self._scan = []
            self._scan_id = scan_id

        raw = np.frombuffer(lidar.raw_data, np.float32).reshape(-1, 4).copy()  # CARLA reuses the buffer
        points = raw[:, :3]
        images = [frame[c.attributes["role_name"]] for c in self.cameras]
        rgb = colorize(points, np.array(lidar.transform.get_matrix()), images)
        channel_counts = [lidar.get_point_count(ch) for ch in range(lidar.channels)]
        ring, t, distance = slice_fields(points, channel_counts, index, lidar.horizontal_angle,
                                         self.ticks_per_scan, self._dt, self._horizontal_fov)
        self._scan.append((index, lidar.timestamp, raw, rgb, ring, t, distance))
        self._send_tf(lidar)
        if index == self.ticks_per_scan - 1:
            self._publish(scan_id)
            self._scan = []

    def _publish(self, scan_id):
        """Publishes the slices of the turn as PointCloud2 (right-handed: y flipped), stamped at its start."""
        if len(self._scan) < self.ticks_per_scan:
            self.node.logwarn("{}: turn {} published with {}/{} slices".format(
                self.name, scan_id, len(self._scan), self.ticks_per_scan))
        index, timestamp = self._scan[0][:2]
        raw, rgb, ring, t, distance = (np.concatenate([s[i] for s in self._scan]) for i in range(2, 7))
        cloud = np.zeros(len(raw), POINT_DTYPE)  # reflectivity and ambient are not modeled: 0 as in ouster-ros
        cloud['x'], cloud['y'], cloud['z'], cloud['intensity'] = raw[:, 0], -raw[:, 1], raw[:, 2], raw[:, 3]
        cloud['t'], cloud['ring'], cloud['range'] = t, ring, distance
        cloud['rgb'] = rgb

        # start of the turn, also when its first slices were dropped (ouster-ros extrapolates it as well)
        header = self.get_msg_header(timestamp=timestamp - index * self._dt)
        self.publisher.publish(PointCloud2(header=header, height=1, width=len(cloud), fields=FIELDS,
                                           is_bigendian=False, point_step=POINT_DTYPE.itemsize,
                                           row_step=POINT_DTYPE.itemsize * len(cloud), is_dense=True,
                                           data=cloud.tobytes()))

    def _send_tf(self, lidar):
        """Sends TF world_frame -> LiDAR frame at the slice of the LiDAR measurement."""
        pose = trans.carla_transform_to_ros_pose(lidar.transform)
        transform = TransformStamped()
        transform.header = self.get_msg_header(frame_id=self.node.parameters['world_frame'],
                                               timestamp=lidar.timestamp)
        transform.child_frame_id = self.get_prefix()
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self._tf_broadcaster.sendTransform(transform)
