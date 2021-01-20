#!/usr/bin/env python

import sys
import copy
import rospy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math
import json 
import message_filters
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Pose
from math import pi
from math import cos
from math import sin
from math import radians
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from moveit_commander.conversions import pose_to_list
from ur_library import all_close, MoveGroupPythonInteface
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from tf.transformations import *
from geometry_msgs.msg import Quaternion

class Transformator:
	def __init__(self, m1_t, m2_t, m3_t, m4_t):
		self.marker_0 = m1_t
		self.marker_1 = m2_t
		self.marker_4 = m3_t
		self.marker_5 = m4_t

matrix_1_size = [10,10]
matrix_2_size = [8,12]
diameter_1 = 0.0127
diameter_2 = 0.006
zmienna = 0
isHome = True
grasp_goal = [Pose(),Pose(),Pose(),Pose()]
markers_ABS_poses = Transformator([0.023,0.1,0.068],[0,0,0],[0,0,0],[0,0,0])
marker_transform = Transformator([0.013,-0.0015,0.047],[0,0,0],[0,0,0],[0,0,0])
z_orient = quaternion_from_euler(0,0,0,axes='sxyz')

 

def getTowerPosition(vlen,angle):
	transform = [vlen*cos(angle),vlen*sin(angle)]
	return transform



def spiral(move_group,angleToStep, startRadius, stepSize=0.0005,minAngle=0.0, minRadius=0.0):
	r = startRadius
	x = 0
	z= 0
	y =0
	pos = [0,0,0,0,0,0]
	o = move_group.move_group.get_current_pose().pose.position #get_actual_tcp_pose()
	startPos = [o.x,o.y,o.z]
	phi = 720 #-angleToStep
	
	while phi > minAngle and r > minRadius:
		phi=phi-angleToStep
    		x=cos(radians(phi))*r
    		y=sin(radians(phi))*r
    
    		pos[0]=x
    		pos[1]=y
    		pos[2]=pos[2]+0.001
    		print("x: %.3f, y: %.3f, xadd: %.3f, yadd: %.3f" %(startPos[0],startPos[1], pos[0], pos[1]))
    		r=r-stepSize
    		if r >=minRadius:
        		plan = move_group.plan_path_to_goal(startPos[0]+pos[0],
    			startPos[1]+pos[1],
    			startPos[2]-pos[2],
    			0,
    			0,
    			0,
    			1)
        		move_group.execute_plan(plan)
    		else:
    			print("Finish")

def matrixMovement(move_group, angle, diameter, size, z_trans):
        global matrix_size
	long_angle = angle - pi/2.0
	counter = 0
	multiplicator = 1
	
	for i in range(size[1]):
		current_pos = move_group.move_group.get_current_pose().pose.position
		long_trans = getTowerPosition(diameter, long_angle)
		current_pos.x += long_trans[0]
		current_pos.y += long_trans[1]	
		plan = move_group.plan_path_to_goal(current_pos.x,
		current_pos.y,
		current_pos.z,
    		z_orient[0],
    		z_orient[1],
    		z_orient[2],
    		z_orient[3])
		move_group.execute_plan(plan)
		trans = getTowerPosition(diameter, angle)	
		for j in range(size[0]):
			try:
			 if counter%2==0 and counter!=0:
				multiplicator = 1
			 elif counter%2!=0:
				multiplicator = -1
			 current_pos = move_group.move_group.get_current_pose().pose.position
			
			 current_pos.x += multiplicator*trans[0]
			 current_pos.y += multiplicator*trans[1]	
			 plan = move_group.plan_path_to_goal(current_pos.x,
			 current_pos.y,
			 current_pos.z,
    			 z_orient[0],
    			 z_orient[1],
   			 z_orient[2],
   			 z_orient[3])
			 move_group.execute_plan(plan)
			except KeyboardInterrupt:
         		 print('Interrupted')	
	 		 try:
        		  sys.exit(0)
         		 except SystemExit:
        		  os._exit(0)		
		counter +=1
	
			
			

def callback(q,w,e,r,t):
	
	global grasp_goal
	grasp_goal[0] = q.pose
	grasp_goal[1] = w.pose
	grasp_goal[2] = e.pose
	grasp_goal[3] = r.pose
	grasp_goal[4] = t.pose
	
	


def callback2(msg):
	
	global grasp_goal
	grasp_goal[1] = msg.pose
	

def showMarkerPositions():
	counter = 1
        for i in grasp_goal:
		print("Marker_%d: x:%.3f, y:%.3f, z:%.3f, xo:%.3f, yo:%.3f, zo:%.3f, wo:%.3f"%(counter, i.position.x, i.position.y,i.position.z, i.orientation.x,i.orientation.y,i.orientation.z,i.orientation.w))
		counter+=1

def getMarkerPositions():
	
	
	#markers_id = [int(x) for x in raw_input("Enter multiple value: ").split(",")]
	#m1,m2,m3,m4,m5 = markers_id
	m1 = input("Enter tips id:")
	m2 = input("Enter test-tube_1 id:")
	m3 = input("Enter test-tube_2 id:")
	m4 = input("Enter trash-can id:")
	#m5 = input("Enter todo id:")

	m1_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m1), PoseStamped)
	m2_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m2), PoseStamped)
	m3_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m3), PoseStamped)
	m4_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m4), PoseStamped)
	#m5_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m5), PoseStamped)
	
	ts = message_filters.TimeSynchronizer([m1_sub,m2_sub,m3_sub,m4_sub],10) 
	ts.registerCallback(callback)
	#rospy.spin()
	#sub = rospy.Subscriber("/aruco_single%d/pose"%(m1), PoseStamped, callback1, (5,8))
	#sub = rospy.Subscriber("/aruco_single%d/pose"%(m2), PoseStamped, callback2)

def startPath(move_group):
    
    global markers_ABS_poses
    global marker_transform
    global z_orient
    markers_ABS_poses.marker_8[0] = grasp_goal[0].position.x
    markers_ABS_poses.marker_8[1] = grasp_goal[0].position.y
    markers_ABS_poses.marker_8[2] = grasp_goal[0].position.z
    markers_ABS_poses.marker_400[0] = grasp_goal[1].position.x
    markers_ABS_poses.marker_400[1] = grasp_goal[1].position.y
    markers_ABS_poses.marker_400[2] = grasp_goal[1].position.z

    #showMarkerPositions()
    
    base_pose = move_group.move_group.get_current_pose().pose
    q_orig = [grasp_goal[0].orientation.x, grasp_goal[0].orientation.y, grasp_goal[0].orientation.z, grasp_goal[0].orientation.w]
    q_rot = quaternion_from_euler(1.5705,-1.5705,-1.5705,axes='sxyz')
    q_marker = euler_from_quaternion(q_orig)
    q_orig_new_quat = quaternion_from_euler(q_marker[0]-1.5705,q_marker[1],q_marker[2]+3.141)
    q_orig_new_euler = euler_from_quaternion(q_orig_new_quat)
    #q_new = quaternion_multiply(q_orig,q_rot)
    print("The new quaternion representation is %.4f %.4f %.4f %.4f." % (q_orig_new_quat[0],q_orig_new_quat[1],q_orig_new_quat[2],q_orig_new_quat[3]))
    print("The old quaternion representation is %.4f %.4f %.4f %.4f." % (q_orig[0],q_orig[1],q_orig[2],q_orig[3]))
    print("The euler new quaternion representation is %.4f %.4f %.4f." % (q_orig_new_euler[0],q_orig_new_euler[1],q_orig_new_euler[2]))
    marker1_angle = q_orig_new_euler[2] #- 0.2146
    ee_angle = euler_from_quaternion([base_pose.orientation.x,base_pose.orientation.y,base_pose.orientation.z,base_pose.orientation.w])[2]

    trans = getTowerPosition(0.10,(pi/2.0 + marker1_angle))
    #trans_diff = getTowerPosition(0.0,(pi-ee_angle))
    
    if (marker1_angle*180.0/pi) >= 45:
	z_orient = quaternion_from_euler(0,0,pi/4.0,axes='sxyz')
    elif (marker1_angle*180.0/pi) <= -45:
	z_orient = quaternion_from_euler(0,0,-pi/4.0,axes='sxyz')
    else:
	z_orient = quaternion_from_euler(0,0,0,axes='sxyz')

    marker_transform.marker_1[0] = trans[0]
    marker_transform.marker_1[1] = trans[1]
    plan = move_group.plan_path_to_goal(grasp_goal[0].position.x-trans[0]+trans_diff[1],
    grasp_goal[0].position.y+trans[1]+trans_diff[0],
    grasp_goal[0].position.z+ marker_transform.marker_8[2],
    0,
    0,
    0,
    1)
    

    
    move_group.execute_plan(plan)
    base_pose = move_group.move_group.get_current_pose().pose
    plan = move_group.plan_path_to_goal(base_pose.position.x+0.024,
    base_pose.position.y+0.0263, base_pose.position.z,0, 0,0,1)

    move_group.execute_plan(plan)

def choseOption(arg, move_group):
	if arg == 1:
		try:
			#planHome = move_group.plan_path_to_goal(-0.0006, 0.440403,0.26170,0,0, 0,1)
			planHome = move_group.plan_path_to_goal(-0.1311, 0.3638,0.30775,0,0, 0,1)
			move_group.execute_plan(planHome)
			#move_group.go_to_joint_state(61, -285, 87, -194, 305, 182)
			print("Home position achieved")
			global isHome
			isHome = True
		except:
			print("Something went wrong")
	elif arg == 2:
		if isHome:
			getMarkerPositions()
			
		else:
			print("Robot is not in Home position")
	elif arg == 3:
		#try:	
			#startPath(move_group)
            	spiral(move_group, 36, 0.01)
		print("Planned position achieved")
		#except:
			#print("Can not achieve this position")

	elif arg == 4:
			
		#showMarkerPositions()
		matrixMovement(move_group, pi/4.0, diameter_1, matrix_1_size)
	elif arg == 5:
		global zmienna
		zmienna = 5
	

def main():
  
  try:
    
    while zmienna != 5:
        move_group = MoveGroupPythonInteface()
    	print "Enter which movement you want to execute: "
    	print "1. Go to home position"
    	print "2. Find AR tag (you need to be in home position)"
    	print "3. Go to detected tag (you need to have detected tag)"
	print "4. Show markers positions"
	print "5. Exit program"

	
    	option = input("\nYour choice: ")
    	
    	
    	choseOption(option, move_group)
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
