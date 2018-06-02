#!/usr/bin/env python
import numpy
#import pickle
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import tf.transformations
import sys, time
from numpy.linalg import inv


import timeit

from argparse import ArgumentParser

from pyquaternion import Quaternion

import roslaunch
class test:
	def __init__(self):

		self.tag_1_detected_flag = False
		self.tag_2_detected_flag = False
		self.tag_3_detected_flag = False
		self.tag_4_detected_flag = False

		startup_start = timeit.default_timer()
		print('start')

		#Rate init
		self.rate = rospy.Rate(20.0) # MUST be more then 2Hz

		#Rate init
		self.rate_tag_detector = rospy.Rate(200.0) # MUST be more then 2Hz

		#Rate init
		self.setpoint_publish_rate = rospy.Rate(20.0) # MUST be more then 2Hz
		
		#Local setpoint publisher init
		#self.setpoint_raw_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

		self.cam_wrt_tag_1_pub = rospy.Publisher("/aruco_mapper/cam_pose", PoseStamped, queue_size=10)

		#April tag pose subscriber init code
		self.aruco_tag_subscriber1 = rospy.Subscriber("/simple_n/pose",Pose, self.callback_tag_detection1,  queue_size = 1)
		#April tag pose subscriber init code
		self.aruco_tag_subscriber2 = rospy.Subscriber("/simple_n/pose2",Pose, self.callback_tag_detection2,  queue_size = 1)
		#April tag pose subscriber init code
		self.aruco_tag_subscriber3 = rospy.Subscriber("/simple_n/pose3",Pose, self.callback_tag_detection3,  queue_size = 1)
		#April tag pose subscriber init code
		self.aruco_tag_subscriber4 = rospy.Subscriber("/simple_n/pose4",Pose, self.callback_tag_detection4,  queue_size = 1)

		#Target pose publisher using aruco pose
		while not rospy.is_shutdown():

			if self.tag_2_detected_flag == True:
				print('Publishing using tag 2')

				self.tag_2_tag_1_pos_x = self.cam_tag_1_pos_x - self.cam_tag_2_pos_x
				self.tag_2_tag_1_pos_y = self.cam_tag_1_pos_y - self.cam_tag_2_pos_y

				self.cam_tag_1 = PoseStamped()
				self.cam_tag_1.header.stamp = rospy.Time.now()
				self.cam_tag_1.header.frame_id = "home"
				self.cam_tag_1.pose.position.x = self.cam_tag_2_pos_x-self.tag_2_tag_1_pos_x
				self.cam_tag_1.pose.position.y = self.cam_tag_2_pos_x-self.tag_2_tag_1_pos_y
				self.cam_wrt_tag_1_pub.publish(self.cam_tag_1)
			elif self.tag_1_detected_flag == True:
				print('Publishing using tag 1')
				self.cam_tag_1 = PoseStamped()
				self.cam_tag_1.header.stamp = rospy.Time.now()
				self.cam_tag_1.header.frame_id = "home"
				self.cam_tag_1.pose.position.x = self.cam_tag_1_pos_x
				self.cam_tag_1.pose.position.y = self.cam_tag_1_pos_y
				self.cam_wrt_tag_1_pub.publish(self.cam_tag_1)

			self.rate_tag_detector.sleep()

	#Current state subscriber
	def callback_tag_detection1(self,state):
		self.cam_tag_1_pos_x = -state.position.x
		self.cam_tag_1_pos_y = -state.position.y
		#print('Tag 1 callback')
		self.tag_1_detected_flag = True
		
	#Current state subscriber
	def callback_tag_detection2(self,state):
		self.cam_tag_2_pos_x = -state.position.x
		self.cam_tag_2_pos_y = -state.position.y
		self.tag_2_detected_flag = True

	#Current state subscriber
	def callback_tag_detection3(self,state):
		self.cam_tag_3_pos_x = -state.position.x
		self.cam_tag_3_pos_y = -state.position.y
		self.tag_3_detected_flag = True

	#Current state subscriber
	def callback_tag_detection4(self,state):
		self.cam_tag_4_pos_x = -state.position.x
		self.cam_tag_4_pos_y = -state.position.y
		self.tag_4_detected_flag = True

def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
