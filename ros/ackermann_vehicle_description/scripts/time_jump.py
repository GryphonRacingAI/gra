#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2, CompressedImage, Image
from visualization_msgs.msg import MarkerArray
from vision_msgs.msg import Detection3DArray
from ultralytics_ros.msg import YoloResult

print('starting')

def adjust_stamp_and_publish(msg, publisher, msg_stamp_attr='header.stamp'):
    global first_stamp, now
    original_stamp = eval('msg.' + msg_stamp_attr)
    if first_stamp is None:
        first_stamp = original_stamp
    adjusted_stamp = original_stamp - first_stamp + now
    exec('msg.' + msg_stamp_attr + ' = adjusted_stamp')
    publisher.publish(msg)

def cb_pointcloud(msg):
    adjust_stamp_and_publish(msg, pub_pointcloud)
    print(1)

def cb_compressed_image(msg):
    adjust_stamp_and_publish(msg, pub_compressed_image)
    print(2)


def cb_image(msg):
    adjust_stamp_and_publish(msg, pub_image)
    print(3)

def cb_marker_array(msg):
    adjust_stamp_and_publish(msg, pub_marker_array)
    print(4)

def cb_detection_3d_array(msg):
    adjust_stamp_and_publish(msg, pub_detection_3d_array)
    print(5)

def cb_yolo_result(msg):
    adjust_stamp_and_publish(msg, pub_yolo_result)
    print(6)

rospy.init_node('rosbag_sync_node')
first_stamp = None
rospy.sleep(1)
now = rospy.Time.now()

# Publishers
pub_pointcloud = rospy.Publisher('/velodyne_points_sync', PointCloud2, queue_size=1)
pub_compressed_image = rospy.Publisher('/camera/aligned_depth_to_color/image_raw/compressedDepth_sync', CompressedImage, queue_size=1)
# pub_image = rospy.Publisher('/yolo_image_sync', Image, queue_size=1)
# pub_marker_array = rospy.Publisher('/detection_marker_sync', MarkerArray, queue_size=1)
pub_detection_3d_array = rospy.Publisher('/yolo_3d_result_sync', Detection3DArray, queue_size=1)
pub_yolo_result = rospy.Publisher('/yolo_result_sync', YoloResult, queue_size=1)

# Subscribers
sub_pointcloud = rospy.Subscriber('/velodyne_points', PointCloud2, cb_pointcloud)
sub_compressed_image = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw/compressedDepth', CompressedImage, cb_compressed_image)
# sub_image = rospy.Subscriber('/yolo_image', Image, cb_image)
# sub_marker_array = rospy.Subscriber('/detection_marker', MarkerArray, cb_marker_array)
sub_detection_3d_array = rospy.Subscriber('/yolo_3d_result', Detection3DArray, cb_detection_3d_array)
sub_yolo_result = rospy.Subscriber('/yolo_result', YoloResult, cb_yolo_result)

rospy.spin()
