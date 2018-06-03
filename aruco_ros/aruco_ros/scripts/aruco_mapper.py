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
		self.tag_2_diff_calculated_flag = False
		self.tag_1_pos_calculated_flag = False

		startup_start = timeit.default_timer()
		print('start')

		#Rate init
		self.rate = rospy.Rate(200.0) # MUST be more then 2Hz

		#Rate init
		self.rate_tag_detector = rospy.Rate(200.0) # MUST be more then 2Hz

		#Rate init
		self.setpoint_publish_rate = rospy.Rate(20.0) # MUST be more then 2Hz
		
		#Local setpoint publisher init
		#self.setpoint_raw_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=10)

		self.cam_wrt_tag_1_pub = rospy.Publisher("/aruco_mapper/unfiltered_cam_pose", PoseStamped, queue_size=10)

		#April tag pose subscriber init code
		self.aruco_tag_subscriber1 = rospy.Subscriber("/simple_n/pose",Pose, self.callback_tag_detection1,  queue_size = 1)
		#April tag pose subscriber init code
		self.aruco_tag_subscriber2 = rospy.Subscriber("/simple_n/pose2",Pose, self.callback_tag_detection2,  queue_size = 1)
		
		#Target pose publisher using aruco pose
		while not rospy.is_shutdown():
			if self.tag_2_detected_flag==True and self.tag_1_pos_calculated_flag==True:
				print('\n')

				print('USING TAG 2:')
				print('Position of cam wrt tag 2:')
				print(self.cam_tag_2_pos_x)
				print(self.cam_tag_2_pos_y)
				print(self.cam_tag_2_pos_z)
				
				print('\n')
				print('Position of tag2 wrt tag 1:')
				print(self.tag2_tag1[0])
				print(self.tag2_tag1[1])
				print(self.tag2_tag1[2])
				
				print('\n')
				print('Position of cam wrt tag 1:')
				print(self.cam_tag_1_pos_x)
				print(self.cam_tag_1_pos_y)
				print(self.cam_tag_1_pos_z)

				self.cam_pose = PoseStamped()
				self.cam_pose.header.stamp = rospy.Time.now()
				self.cam_pose.header.frame_id = "home"
				self.cam_pose.pose.position.x = self.cam_tag_1_pos_x
				self.cam_pose.pose.position.y = self.cam_tag_1_pos_y
				self.cam_pose.pose.position.z = self.cam_tag_1_pos_z
				self.cam_pose.pose.orientation = self.tag_2_orientation
				self.cam_wrt_tag_1_pub.publish(self.cam_pose)


			elif self.tag_1_detected_flag==True:
				print('USING TAG 1:')
				print('Position of cam wrt tag 1:')
				print(self.cam_tag_1_pos_x)
				print(self.cam_tag_1_pos_y)
				print(self.cam_tag_1_pos_z)
				print('\n')
				self.cam_pose = PoseStamped()
				self.cam_pose.header.stamp = rospy.Time.now()
				self.cam_pose.header.frame_id = "home"
				self.cam_pose.pose.position.x = self.cam_tag_1_pos_x
				self.cam_pose.pose.position.y = self.cam_tag_1_pos_y
				self.cam_pose.pose.position.z = self.cam_tag_1_pos_z
				self.cam_pose.pose.orientation = self.tag_1_orientation
				self.cam_wrt_tag_1_pub.publish(self.cam_pose)
			self.rate.sleep()

	#Current state subscriber
	def callback_tag_detection1(self,state):
		self.tag_1_orientation = state.orientation
		self.tag_1_pos_x = state.position.x
		self.tag_1_pos_y = state.position.y
		self.tag_1_pos_z = state.position.z
		
		
		self.tag1_cam_quat = Quaternion([self.tag_1_orientation.w, self.tag_1_orientation.x, self.tag_1_orientation.y, self.tag_1_orientation.z])

		self.tag1_cam_matrix = self.tag1_cam_quat.transformation_matrix

		self.tag1_cam_matrix[0][3] = state.position.x
		self.tag1_cam_matrix[1][3] = state.position.y
		self.tag1_cam_matrix[2][3] = state.position.z

		self.cam_tag1_matrix = inv(self.tag1_cam_matrix)

		#print(self.cam_tag1_matrix)

		self.cam_tag_1_pos_x = self.cam_tag1_matrix[0][3]
		self.cam_tag_1_pos_y = self.cam_tag1_matrix[1][3]
		self.cam_tag_1_pos_z = self.cam_tag1_matrix[2][3]
		if self.tag_2_detected_flag==True:
			self.tag2_tag1 = [self.cam_tag_2_pos_x-self.cam_tag_1_pos_x,self.cam_tag_2_pos_y-self.cam_tag_1_pos_y,self.cam_tag_2_pos_z-self.cam_tag_1_pos_z]
			self.tag_2_diff_calculated_flag=True
		self.tag_1_detected_flag = True

	def callback_tag_detection2(self,state):
		self.tag_2_orientation = state.orientation
		self.tag_2_pos_x = state.position.x
		self.tag_2_pos_y = state.position.y
		self.tag_2_pos_z = state.position.z
		
		
		self.tag2_cam_quat = Quaternion([self.tag_2_orientation.w, self.tag_2_orientation.x, self.tag_2_orientation.y, self.tag_2_orientation.z])
		
		self.tag2_cam_matrix = self.tag2_cam_quat.transformation_matrix

		self.tag2_cam_matrix[0][3] = state.position.x
		self.tag2_cam_matrix[1][3] = state.position.y
		self.tag2_cam_matrix[2][3] = state.position.z

		self.cam_tag2_matrix = inv(self.tag2_cam_matrix)

		#print(self.cam_tag2_matrix)

		self.cam_tag_2_pos_x = self.cam_tag2_matrix[0][3]
		self.cam_tag_2_pos_y = self.cam_tag2_matrix[1][3]
		self.cam_tag_2_pos_z = self.cam_tag2_matrix[2][3]
		if self.tag_2_diff_calculated_flag==True:
			self.cam_tag_1_pos_x = self.cam_tag_2_pos_x - self.tag2_tag1[0]
			self.cam_tag_1_pos_y = self.cam_tag_2_pos_y - self.tag2_tag1[1]
			self.cam_tag_1_pos_z = self.cam_tag_2_pos_z - self.tag2_tag1[2]
			self.tag_1_pos_calculated_flag=True

			

		self.tag_2_detected_flag = True

def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
