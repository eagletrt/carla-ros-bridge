#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
One-shot helper that requests sensor.pseudo.rgb_lidar from the running
carla_ros_bridge, via its /carla/spawn_object service, attached to the vehicle
carrying the rgb_lidar LiDAR. Run it once after the bridge is up and the vehicle
with its RGB LiDAR rig is spawned. The CARLA server is read from the parameters
host, port and timeout, as for the bridge.
"""

import sys

import carla
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.srv import SpawnObject
from geometry_msgs.msg import Pose

from carla_ros_bridge.rgb_lidar_sensor import LIDAR_ROLE, RgbLidarSensor


def _find_rig_vehicle_id(host, port, timeout):
    """Id of the vehicle the rgb_lidar LiDAR is attached to."""
    client = carla.Client(host, port)
    client.set_timeout(timeout)
    world = client.get_world()
    # in synchronous mode the actor registry is filled only after a tick is observed
    world.wait_for_tick(seconds=timeout)
    lidars = [a for a in world.get_actors().filter("sensor.lidar.ray_cast")
              if a.attributes.get("role_name") == LIDAR_ROLE and a.parent is not None]
    if not lidars:
        raise RuntimeError("No '{}' LiDAR attached to a vehicle - spawn the rig first".format(LIDAR_ROLE))
    return lidars[0].parent.id


class SpawnRgbLidar(CompatibleNode):

    def __init__(self):
        super(SpawnRgbLidar, self).__init__("spawn_rgb_lidar")
        self.spawn_object_client = self.new_client(SpawnObject, "/carla/spawn_object")

    def run(self):
        vehicle_id = _find_rig_vehicle_id(self.get_param('host', 'localhost'), self.get_param('port', 2000),
                                          float(self.get_param('timeout', 5.0)))
        request = roscomp.get_service_request(SpawnObject)
        request.type = RgbLidarSensor.get_blueprint_name()
        request.id = LIDAR_ROLE
        request.attach_to = vehicle_id
        request.transform = Pose()
        request.transform.orientation.w = 1.0
        request.random_pose = False
        response = self.call_service(self.spawn_object_client, request, spin_until_response_received=True)
        if response.id == -1:
            raise RuntimeError("Failed to spawn '{}': {}".format(request.type, response.error_string))
        self.loginfo("Spawned '{}' (id={}) on vehicle id={}".format(request.type, response.id, vehicle_id))


def main(args=None):
    roscomp.init("spawn_rgb_lidar", args=args)
    node = None
    try:
        node = SpawnRgbLidar()
        node.run()
    except (RuntimeError, IndexError) as exc:
        if node:
            node.logerr(str(exc))
        else:
            print(exc, file=sys.stderr)
        sys.exit(1)
    finally:
        if node:
            node.destroy_node()
        roscomp.shutdown()


if __name__ == "__main__":
    main()
