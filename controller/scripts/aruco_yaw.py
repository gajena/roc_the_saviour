#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import tf.transformations
import numpy as np

def tag_detect_node():

	global aruco_inverted_pose_pub
	rospy.init_node('aruco_transform', anonymous=True)
	aruco_inverted_pose_pub = rospy.Publisher("/aruco_single/transformed_pose", PoseStamped, queue_size=10)
	aruco_tag_sub = rospy.Subscriber("/aruco_single/pose",PoseStamped, tag_detection_cb)
	rospy.spin()

def tag_detection_cb(msg):
	trans_matrix = tf.transformations. quaternion_matrix([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
	angles = tf.transformations.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w], axes='sxyz')
	print(angles)

if __name__ == '__main__':
	try:
		tag_detect_node()
	except rospy.ROSInterruptException:
		pass
