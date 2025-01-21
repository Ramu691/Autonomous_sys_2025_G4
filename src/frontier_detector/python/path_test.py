#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point,PoseStamped, TwistStamped
from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry

horizon_distance = 20 #meters

depth_image = np.zeros((240,320,1))
goal_point = np.array([[0],[0],[0]])
goal_point_w = np.array([[0],[0],[0]])

Rd = np.zeros((3,3))
t = np.zeros((3,1))
R = np.zeros((3,3))
xd = np.zeros((3,1))
tc = np.array([[0.1],[0],[0]])
X = np.array([[0],[0],[0]])

drone_pose = PoseStamped()
drone_twist = TwistStamped()

cam_info = CameraInfo()

Rc = np.array([[ 0.0,  0.0, 1.0],
	     	   [-1.0,  0.0, 0.0],
	     	   [0.0, -1.0, 0.0]]);
	     	 
def cam_inf_callback(msg):
	global cam_info
	cam_info = msg

def to_dsp():
	global drone_pose
	global drone_twist
	
	odom_msg = Odometry()
	odom_msg.header.frame_id = "world"
	odom_msg.pose.pose = drone_pose.pose
	odom_msg.twist.twist = drone_twist.twist
	odom_pub.publish(odom_msg)
	
	start_point_msg = Point()
	start_point_msg.x = drone_pose.pose.position.x
	start_point_msg.y = drone_pose.pose.position.y
	start_point_msg.z = drone_pose.pose.position.z
	dsp_start_pub.publish(start_point_msg)
	
def onTwist(msg):
	global drone_twist
	drone_twist = msg
	#print(math.sqrt(msg.twist.angular.x**2 + msg.twist.angular.z**2 + msg.twist.angular.z**2))

def argsort(seq):
    return sorted(range(len(seq)), key=seq.__getitem__)
   
def transformation_matrix(rotation_matrix, translation_vector):
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix
    translation_vector = np.squeeze(translation_vector)
    transformation_matrix[:3, 3] = translation_vector 
    return transformation_matrix
    
def onCurrentState(msg):
	global R
	global t
	global X
	global drone_pose
	
	drone_pose = msg
	
	x = msg.pose.position.x
	y = msg.pose.position.y
	z = msg.pose.position.z
	
	X = np.array([[x],[y],[z]])
	
	xd = np.array([[x],[y],[z]])
	
	q0 = msg.pose.orientation.x
	q1 = msg.pose.orientation.y
	q2 = msg.pose.orientation.z
	q3 = msg.pose.orientation.w

	Rd = np.array([[q0**2-q1**2-q2**2+q3**2, 2*(q0*q2 - q2*q3), 2*(q0*q2 + q1*q3)],
        		   [2*(q0*q1 + q2*q3), -q0**2+q1**2-q2**2+q3**2, 2*(q1*q2 - q0*q3)],
        		   [2*(q0*q2 - q1*q3), 2*(q1*q2 + q0*q3), -q0**2-q1**2+q2**2+q3**2]])
        
    
	#print(Rd)
	Td = transformation_matrix(Rd,xd)
	Tc = transformation_matrix(Rc,tc)
	T = np.matmul(Td,Tc)
	
	R = T[:3,:3]
	t = T[:3, -1]
	t = t.reshape(-1,1)
	

def publish_frontier_depth(depth_image):
	global cam_info
	depth_image = depth_image.copy()
	
	frontier = np.zeros((240,320),dtype=np.uint16)
	new_depth_image = depth_image.copy()
	
	
	max_range = rospy.get_param("/octomap_server/sensor_model/max_range")
	horizon_depth = 255/60*max_range
	
	frontier[depth_image==0] = 50000

	new_depth_image = depth_image + frontier#cv2.bitwise_and(depth_image, depth_image, mask = frontier)
	new_depth_image = new_depth_image#*(50000/255)
	
	
	bridge = CvBridge()
	frontier_msg = bridge.cv2_to_imgmsg(frontier.astype(np.uint16), encoding='mono16')
	frontier_msg.header = cam_info.header
	frontier_pub.publish(frontier_msg)
	
	new_depth_msg = bridge.cv2_to_imgmsg(new_depth_image.astype(np.uint16), encoding='mono16')
	new_depth_msg.header = cam_info.header
	new_depth_pub.publish(new_depth_msg)
	cam_info_pub.publish(cam_info)
	


def depth_callback(msg):
	global depth_image
	global R
	global t
	global goal_point_w
	global goal_point
	bridge = CvBridge()
	msg.encoding = "mono16"
	depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
	
	#publish_frontier_depth(depth_image)
	
	depth_image = cv2.convertScaleAbs(depth_image, alpha=(255.0/65535.0))

	mask = np.zeros((240,320),dtype=np.uint8)
	
	depth_image[depth_image>255] = 0
	
	mask[depth_image==0] = 255

	contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

	if(len(contours) > 0):
		areas = list()
		cont_x = list()
		cont_y = list()

		for i,cnt in enumerate(contours):
			M = cv2.moments(cnt)
			x_bb,y_bb,w_bb,h_bb = cv2.boundingRect(cnt)
			print("w/h ratio " + str(i) + " " + str(w_bb/h_bb))
			if(M['m00'] != 0 and cv2.contourArea(cnt) > 3000 and w_bb/h_bb > 0.5):
				areas.append(cv2.contourArea(cnt))
				cont_x.append(int(M['m10']/M['m00']))
				cont_y.append(int(M['m01']/M['m00']))
		
		if(len(areas) > 0):	
			areas_index = argsort(areas)
				
			cx = cont_x[areas_index[len(areas_index)-1]]
			cy = cont_y[areas_index[len(areas_index)-1]]
			goal_point[2] = 20
			goal_point[0] = (cx-160)*goal_point[2] / 120
			goal_point[1] = (cy-120)*goal_point[2] / 120
			
			cv2.circle(mask,(cx,cy),1,(0,255,0),1)

			goal_point_w = np.matmul(R,goal_point) + t

	
	
	cv2.imshow("mask", mask)
	cv2.waitKey(1)
	

	
def vector_to_euler(x,y,z):
    yaw = np.arctan2(y, x)
    pitch = np.arctan2(-z, np.sqrt(x**2 + y**2))
    roll = np.arctan2(np.sin(pitch)*y - np.cos(pitch)*x, -z)

    return roll, pitch, yaw

def timer_callback(event):
	global X
	pose_msg = PoseStamped()
	pose_msg.header.frame_id = "world"
	pose_msg.pose.position.x = goal_point_w[0]
	pose_msg.pose.position.y = goal_point_w[1]
	pose_msg.pose.position.z = goal_point_w[2]
	
	roll,pitch,yaw = vector_to_euler(goal_point_w[0]-X[0],goal_point_w[1]-X[1],goal_point_w[2]-X[2])
	quat = quaternion_from_euler(0, 0, yaw)  # Replace roll, pitch, yaw with your angles
	pose_msg.pose.orientation.x = quat[0]
	pose_msg.pose.orientation.y = quat[1]
	pose_msg.pose.orientation.z = quat[2]
	pose_msg.pose.orientation.w = quat[3]
	
	to_dsp()
	
	goal_point_msg = Point()
	goal_point_msg.x = -1018.45	#pose_msg.pose.position.x
	goal_point_msg.y = -250.554#pose_msg.pose.position.y
	goal_point_msg.z = -18.7135	#pose_msg.pose.position.z
	dsp_goal_pub.publish(goal_point_msg)
	goal_pub.publish(pose_msg)

	
if __name__ == "__main__":
	rospy.init_node("path_test")
	rospy.Subscriber("/realsense/depth/image", Image, depth_callback)
	rospy.Subscriber("/pose_est", PoseStamped, onCurrentState)
	rospy.Subscriber("/twist_est", TwistStamped, onTwist)
	goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
	odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
	dsp_start_pub = rospy.Publisher("/dsp/set_start", Point, queue_size=10)
	dsp_goal_pub = rospy.Publisher("/dsp/set_goal", Point, queue_size=10)
	
	new_depth_pub = rospy.Publisher("/new_depth", Image, queue_size = 10)
	frontier_pub = rospy.Publisher("/frontier", Image, queue_size=10)
	cam_info_sub = rospy.Subscriber("/realsense/depth/camera_info", CameraInfo, cam_inf_callback)
	cam_info_pub = rospy.Publisher("/new_cam_info", CameraInfo, queue_size=10)
	
	rospy.Timer(rospy.Duration(2), timer_callback)

	
	rospy.spin()

