#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a cone ground truth sensor
"""

from carla_ros_bridge.actor import Actor
from carla_ros_bridge.pseudo_actor import PseudoActor

from carla_msgs.msg import CarlaGroundTruthCone, CarlaGroundTruthConeArray

# CARLA blueprint id (lowercase, no separators) -> ground truth color enum.
# Static cone props are never reclassified at runtime, so a fixed lookup is enough.
_CONE_COLOR_BY_BLUEPRINT = {
    "staticpropbluecone": CarlaGroundTruthCone.BLUE,
    "staticpropblue_cone": CarlaGroundTruthCone.BLUE,
    "staticpropyellowcone": CarlaGroundTruthCone.YELLOW,
    "staticpropyellow_cone": CarlaGroundTruthCone.YELLOW,
    "staticproporangecone": CarlaGroundTruthCone.ORANGE,
    "staticproporange_cone": CarlaGroundTruthCone.ORANGE,
    "staticpropredcone": CarlaGroundTruthCone.RED,
    "staticpropred_cone": CarlaGroundTruthCone.RED,
}


def _cone_color(type_id):
    key = type_id.replace(".", "").replace("-", "").lower()
    return _CONE_COLOR_BY_BLUEPRINT.get(key, CarlaGroundTruthCone.UNKNOWN)


class ConeGroundTruthSensor(PseudoActor):

    """
    Pseudo sensor exposing the exact world-frame position of every spawned
    track cone, bypassing perception entirely. Cones are static props (not
    Vehicle/Walker), so they are invisible to sensor.pseudo.objects.
    """

    def __init__(self, uid, name, parent, node, actor_list):
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
        :param actor_list: current list of actors
        :type actor_list: map(carla-actor-id -> python-actor-object)
        """
        super(ConeGroundTruthSensor, self).__init__(uid=uid,
                                                     name=name,
                                                     parent=parent,
                                                     node=node)
        self.actor_list = actor_list
        self.cone_publisher = node.new_publisher(CarlaGroundTruthConeArray,
                                                 self.get_topic_prefix(),
                                                 qos_profile=10)

    def destroy(self):
        """
        Function to destroy this object.
        :return:
        """
        super(ConeGroundTruthSensor, self).destroy()
        self.actor_list = None
        self.node.destroy_publisher(self.cone_publisher)

    @staticmethod
    def get_blueprint_name():
        """
        Get the blueprint identifier for the pseudo sensor
        :return: name
        """
        return "sensor.pseudo.cone_ground_truth"

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        Publishes the world-frame (map) position of every known cone actor.
        :return:
        """
        ros_cones = CarlaGroundTruthConeArray()
        ros_cones.header = self.get_msg_header(frame_id="map", timestamp=timestamp)

        for actor in self.actor_list.values():
            if not isinstance(actor, Actor):
                continue
            type_id = actor.carla_actor.type_id
            if "cone" not in type_id:
                continue

            position = actor.get_current_ros_pose().position
            cone = CarlaGroundTruthCone()
            cone.id = actor.get_id()
            cone.x = position.x
            cone.y = position.y
            cone.z = position.z
            cone.color = _cone_color(type_id)
            ros_cones.cones.append(cone)

        self.cone_publisher.publish(ros_cones)
