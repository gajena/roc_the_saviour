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
	#print(trans_matrix)
	trans_matrix[0][3] = msg.pose.position.x
	trans_matrix[1][3] = msg.pose.position.y
	trans_matrix[2][3] = msg.pose.position.z
	#print("\nRaw mat ")
	#print(trans_matrix)
	temp = np.asarray(trans_matrix)
	inv_trans_mat = np.linalg.inv(temp)
	#print("\nInv mat ")
	#print(inv_trans_mat)

	inverted_pose = PoseStamped()
	inverted_pose.header.stamp = rospy.Time.now()
	inverted_pose.header.frame_id = "home"
	inverted_pose.pose.position.x = inv_trans_mat[2][3]#z
	inverted_pose.pose.position.y = inv_trans_mat[0][3]#x
	inverted_pose.pose.position.z = -inv_trans_mat[1][3]#y
	aruco_inverted_pose_pub.publish(inverted_pose)



if __name__ == '__main__':
	try:
		tag_detect_node()
	except rospy.ROSInterruptException:
		pass
