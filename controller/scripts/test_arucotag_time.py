#!/usr/bin/env python

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from aruco_mapping.msg import ArucoMarker
import rospy
import timeit

global tag_detector_subscriber_prev
tag_detector_subscriber_prev=0

def tag_detect_node():

	global april_tag_detected_pub
	global aruco_rate_pub
	rospy.init_node('arucotag_detect_node', anonymous=True)
	aruco_rate_pub = rospy.Publisher("/aruco_rate", PoseStamped, queue_size=10)
	aruco_tag_sub = rospy.Subscriber("/aruco_single/pose",PoseStamped, tag_detection_cb)
	rospy.spin()

def tag_detection_cb(msg):
	global tag_detector_subscriber_prev
	print('tag found')
	tag_detector_subscriber_current = timeit.default_timer()
	print('Time between aruco tag detection subscriptions is:')
	time_diff = tag_detector_subscriber_current-tag_detector_subscriber_prev
	print(str(time_diff))
	print('Rate(Hz) = ')
	rate = 1/float(time_diff)
	print(rate)
	rate_1 = PoseStamped()
	rate_1.header.stamp = rospy.Time.now()
	rate_1.header.frame_id = "home"
	rate_1.pose.position.x = rate
	aruco_rate_pub.publish(rate_1)
	tag_detector_subscriber_prev = tag_detector_subscriber_current
if __name__ == '__main__':
	try:
		tag_detect_node()
	except rospy.ROSInterruptException:
		pass
