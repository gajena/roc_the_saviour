#!/usr/bin/env python

import os
import time
import numpy
#import pickle
import rospy
import mavros
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State,AttitudeTarget
from mavros_msgs.srv import CommandBool, SetMode

from std_msgs.msg import Bool,Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf.transformations
import sys



import timeit

from argparse import ArgumentParser

from pyquaternion import Quaternion

import roslaunch

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

class test:
	def __init__(self):

		#Rate init
		self.rate = rospy.Rate(2.0) # MUST be more then 2Hz
		
		#Current state sub
		self.state_sub = rospy.Subscriber("/gripper/position", Int32, self.gripper_position_callback)


		self.grip_status_pub = rospy.Publisher("/gripper/grip_status", Int32, queue_size=10)

		# self.grip_status = 0

		if os.name == 'nt':
			import msvcrt
			def getch():
				return msvcrt.getch().decode()
		else:
			import sys, tty, termios
			fd = sys.stdin.fileno()
			old_settings = termios.tcgetattr(fd)
			def getch():
				try:
					tty.setraw(sys.stdin.fileno())
					ch = sys.stdin.read(1)
				finally:
					termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
				return ch

		# Control table address
		self.ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
		self.ADDR_PRO_GOAL_POSITION      = 116
		self.ADDR_PRO_PRESENT_POSITION   = 132
		self.ADDR_PRO_PRESENT_CURRENT   = 126

		# Protocol version
		self.PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

		# Default setting
		self.DXL_ID                      = 1                 # Dynamixel ID : 1
		self.BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
		self.DEVICENAME                  = '/dev/dynamixel'    # Check which port is being used on your controller
														# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

		self.TORQUE_ENABLE               = 1                 # Value for enabling the torque
		self.TORQUE_DISABLE              = 0                 # Value for disabling the torque

		self.COMM_SUCCESS                = 0


		#0 is open, 1 is close
		self.gripper_position            = 0

		self.gripper_position_raw_open = 1700
		self.gripper_position_raw_close = 4000
		self.gripper_position_raw = self.gripper_position_raw_open



		# Initialize PortHandler instance
		# Set the port path
		# Get methods and members of PortHandlerLinux or PortHandlerWindows
		self.portHandler = PortHandler(self.DEVICENAME)

		# Initialize PacketHandler instance
		# Set the protocol version
		# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
		self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
		
		# Open port
		if self.portHandler.openPort():
			print("Succeeded to open the port")
		else:
			print("Failed to open the port")
			print("Press any key to terminate...")
			getch()
			quit()


		# Set port baudrate
		if self.portHandler.setBaudRate(self.BAUDRATE):
			print("Succeeded to change the baudrate")
		else:
			print("Failed to change the baudrate")
			print("Press any key to terminate...")
			getch()
			quit()

		# Enable Dynamixel Torque
		self.dxl_comm_result, self.dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_TORQUE_ENABLE, self.TORQUE_ENABLE)
		
		if self.dxl_comm_result != self.COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
		elif self.dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))
		else:
			print("Dynamixel has been successfully connected")

		# Write goal position
		self.dxl_comm_result, self.dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_GOAL_POSITION, self.gripper_position_raw_open)
		if self.dxl_comm_result != self.COMM_SUCCESS:
			print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
		elif self.dxl_error != 0:
			print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))


		#Target pose publisher using aruco pose
		while not rospy.is_shutdown():

			# Write goal position
			self.present_current, self.dxl_comm_result, self.dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_PRESENT_CURRENT)
			if self.dxl_comm_result != self.COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
			elif self.dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))

			print("Present current = "+str(self.present_current))

			if self.present_current>=60 and self.present_current<=300:
				self.grip_status = 1
			else:
				self.grip_status = 0

			#print("Grip status = "+str(self.grip_status))	

			self.grip_status_pub.publish(self.grip_status)

			self.rate.sleep()
	
	#Gripper position subscriber
	def gripper_position_callback(self,state):
		#print("Recieved : "+ str(state))
		self.data = state.data
		if self.data==0 or self.data==1:
			self.gripper_position = self.data
			if self.gripper_position==0:
				#print("Position set to open")
				self.gripper_position_raw = self.gripper_position_raw_open
			elif self.gripper_position==1:
				#print("Position set to close")
				self.gripper_position_raw = self.gripper_position_raw_close
			#print('Writing goal position')
			# Write goal position
			self.dxl_comm_result, self.dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.DXL_ID, self.ADDR_PRO_GOAL_POSITION, self.gripper_position_raw)
			if self.dxl_comm_result != self.COMM_SUCCESS:
				print("%s" % self.packetHandler.getTxRxResult(self.dxl_comm_result))
			elif self.dxl_error != 0:
				print("%s" % self.packetHandler.getRxPacketError(self.dxl_error))


def main(args):
	rospy.init_node('offb_node', anonymous=True)
	ic=test()

	try:
		rospy.spin()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)
