#!/usr/bin/env python3
import rospy
from foxglove_msgs.msg import SceneUpdate, SceneEntity, CylinderPrimitive, Color
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from dataclasses import dataclass
from typing import List


@dataclass
class ConeCoords():
    x: float
    y: float


def callback(data: LaserScan, publisher):

    cones = [ConeCoords(-data.ranges[0], 0),
             ConeCoords(0, -data.ranges[len(data.ranges)//4])]
    rospy.loginfo(f"Cones:{cones}")
    publisher.publish(convert_cone_coords_to_foxglove_objects(
        cones, data.header.stamp))


def convert_cone_coords_to_foxglove_objects(coords_list: List[ConeCoords], timestamp):
    update = SceneUpdate()
    for i, object_coords in enumerate(coords_list):
        entity = SceneEntity()
        entity.frame_id = "laser"
        entity.timestamp = timestamp

        entity.frame_locked = True
        entity.id = f"cone{i}"
        entity.cylinders.append(CylinderPrimitive(Pose(
            Point(object_coords.x, object_coords.y, 0.5),
            Quaternion(0, 0, 0, 0)),
            Vector3(0.5, 0.5, 1), 1, 1, Color(1, 1, 0, 1)))

        update.entities.append(entity)
    return update


def listener():

    pub = rospy.Publisher('poses', SceneUpdate, queue_size=10)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback, callback_args=pub)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
