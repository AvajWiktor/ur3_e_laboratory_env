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
from os import system
from geometry_msgs.msg import PoseStamped 
from geometry_msgs.msg import Pose
from math import pi
from math import cos
from math import sin
from math import radians
from math import atan
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
class Point:
	def __init__(self,x,y,z,move_group, fill_trans):
		self.x = x
		self.y = y
		self.z = z
		self.mg = move_group
		self.isEmpty = True
		self.fill_trans = fill_trans
	def goTo(self):
		global z_orient
		plan = self.mg.plan_path_to_goal(self.x,self.y,self.z+self.fill_trans[0],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
		self.mg.execute_plan(plan)
	def fillPoint(self):
		global z_orient
		if self.isEmpty:
			changeState(0, 1)
			plan = self.mg.plan_path_to_goal(self.x,self.y,self.z+self.fill_trans[1],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
			self.mg.execute_plan(plan)
			rospy.sleep(1)
			changeState(0, 0)
			#system("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: 0 \nstate: 1.0\"")
			
			#bp = self.mg.move_group.get_current_pose().pose.position
			#plan = self.mg.plan_path_to_goal(bp.x,bp.y,bp.z+0.07,z_orient[0],z_orient[1],z_orient[2],z_orient[3])
			#self.mg.execute_plan(plan)
			plan = self.mg.plan_path_to_goal(self.x,self.y,self.z+self.fill_trans[2],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
			self.mg.execute_plan(plan)
			self.isEmpty = False
		else:
			print("This prob is full")
	def takeTip(self):
		global z_orient
		if self.isEmpty:
			plan = self.mg.plan_path_to_goal(self.x,self.y,self.z+self.fill_trans[1],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
			self.mg.execute_plan(plan)
			#system("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: 0 \nstate: 1.0\"")
			#bp = self.mg.move_group.get_current_pose().pose.position
			#plan = self.mg.plan_path_to_goal(bp.x,bp.y,bp.z+0.07,z_orient[0],z_orient[1],z_orient[2],z_orient[3])
			plan = self.mg.plan_path_to_goal(self.x,self.y,self.z+self.fill_trans[2],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
			self.mg.execute_plan(plan)
			self.isEmpty = False
		else:
			print("This prob is full")
def changeState(port, state):
	string = ("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: %d \nstate: %d.0\"" %(port,state))
	system(string)

class MatrixProb:
	def __init__(self, angle, diameter, size, first_element, move_group,ft):
		self.angle = angle #angle of aruco marker
		self.diameter = diameter #diameter of glory hole
		self.size = size #size array for example 2x2 -> [2,2]
		self.first_element = first_element #coordinates of 1st hole
		self.points_vector = []
		long_angle = self.angle - pi/2.0
		counter = 0
		multiplicator = 1
		start_pos = self.first_element
		
		self.points_vector.append(Point(start_pos[0],start_pos[1],start_pos[2],move_group,ft))
		for i in range(self.size[1]):
			if counter != 0:

				long_trans = getMiniTowerPosition(long_angle, self.diameter)
				start_pos[0] += long_trans[0]
				start_pos[1] += long_trans[1]
				self.points_vector.append(Point(start_pos[0],start_pos[1],start_pos[2], move_group,ft))

			trans = getMiniTowerPosition(self.angle, self.diameter)
			for i in range(self.size[0]-1):
			
			

			
				if counter%2==0 and counter!=0:
					multiplicator = 1
				elif counter%2!=0:
					multiplicator = -1
				start_pos[0] += multiplicator*trans[0]
				start_pos[1] += multiplicator*trans[1]
				self.points_vector.append(Point(start_pos[0],start_pos[1],start_pos[2], move_group,ft))
			
			counter+=1
	def goToMatrixPoint(self, point_nr):
		if len(self.points_vector) > 0:
			print(self.points_vector[point_nr-1].x, self.points_vector[point_nr-1].y, self.points_vector[point_nr-1].z)
			self.points_vector[point_nr-1].goTo()
		else:
			print("Points vector is empty!")
	def fillMatrixPoint(self, point_nr):
		if len(self.points_vector) > 0:
			self.points_vector[point_nr-1].fillPoint()
		else:
			print("Points vector is empty!")
		
	
matrix_1 = MatrixProb(0, 0, [0,0], [0,0,0], 0,[0,0,0])
matrix_2 = MatrixProb(0, 0, [0,0], [0,0,0], 0,[0,0,0])
matrix_1_size = [10,10]
matrix_1_points =[]
fill_2_t = [0.01,-0.0085,0.05]#-0.0158,0.05]
fill_1_t = [0.04,0.014,0.03] 
matrix_2_size = [8,12]
matrix_2_points =[]
matrix_1_diameter = 0.0127
matrix_2_diameter = 0.009
w_box_ar_1st_t = [0.057175, 0.057875, 0.0157]
t_box_ar_1st_t = [0.0495, 0.0585, 0.02]
cal_box_ar_1st_t = [0, 0.075, 0.02]
zmienna = 0
isHome = True
t = [0.009,0.009,0.035]
grasp_goal = [Pose(),Pose(),Pose()]
markers_ABS_poses = Transformator([0.0285,0.1,0.068],[0,0,0],[0,0,0],[0,0,0])
z_orient = quaternion_from_euler(0,0.033,0,axes="sxyz")

def apply_vacuum(self):
	rospy.init_node("apply_vacum_node") 
	publisher1 = rospy.Publisher('/ur_hardware_interface/script_command', String, quesize=2)
	try:
   	#first 1 is the pin number and second number is the state of the pin 
    		message = "sec MyProgram():" + "\n" + "set_standard_digital_out(1,1)" + "\n" + "end " + "\n"
    		publisher1.publish(message)  
	except rospy.ServiceException, e:
    		print "Something went wrong: %s"%e

def getTowerPosition(angle, a_t):
	clen = (a_t[0]*a_t[0] + a_t[1]*a_t[1])**0.5
	add_angle  = atan(a_t[0]/a_t[1]) + angle
	
	print("clen =%.4f, add_angle = %.4f" %(clen, add_angle))	
	transform = [clen*cos(add_angle),clen*sin(add_angle)]
	return transform

def getMiniTowerPosition(angle, dist):
	
	transform = [dist*cos(angle),dist*sin(angle)]
	return transform
def add_table():
    REFERENCE_FRAME = '/base_link'
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    #scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', scene)
    rospy.sleep(2)
    
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    

    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.103

    scene.add_box("table", p, (3, 3, 0.02))
    q = quaternion_from_euler(0,0, -0.46, axes="sxyz")
    p.pose.position.x = -0.41
    p.pose.position.y = -0.19
    p.pose.position.z = -0.06
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]



    scene.add_box("right_wall", p, (1, 0.02, 1))

    q = quaternion_from_euler(0,0, 0.46, axes="sxyz")
    p.pose.position.x = 0.38
    p.pose.position.y = -0.20
    p.pose.position.z = -0.06
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]

    scene.add_box("left_wall", p, (1, 0.02, 1))
    
    q = quaternion_from_euler(0,0,0, axes="sxyz")
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = 0.65
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]

    scene.add_box("sufit", p, (3, 3, 0.02))

 
    #scene_pub.publish(scene)


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
 

def checkPos(m_g,m_nr):

	bp = m_g.move_group.get_current_pose().pose
	x =  grasp_goal[m_nr].position.x - bp.position.x
	y = bp.position.y 
	angle =  atan(x/y)
	q = quaternion_from_euler(0,0.033,-angle,axes="sxyz")
	return q
			
def checkAngle(angle):
	if angle > 0 and angle < 0.351:
		return True
	elif angle < 0 and angle > -1.570:
		return True 
	else:
		return False	

def callback0(msg):
	
	global grasp_goal
	grasp_goal[0] = msg.pose
	
	
	#grasp_goal[3] = r.pose
	#grasp_goal[4] = t.pose
	
def callback1(msg):
	
	global grasp_goal
	grasp_goal[1] = msg.pose

def callback4(msg):
	
	global grasp_goal
	grasp_goal[2] = msg.pose
	

def showMarkerPositions():
	counter = 1
        for i in grasp_goal:
		print("Marker_%d: x:%.3f, y:%.3f, z:%.3f, xo:%.3f, yo:%.3f, zo:%.3f, wo:%.3f"%(counter, i.position.x, i.position.y,i.position.z, i.orientation.x,i.orientation.y,i.orientation.z,i.orientation.w))
		counter+=1

def getMarkerPositions():
	
 try:
	#markers_id = [int(x) for x in raw_input("Enter multiple value: ").split(",")]
	#m1,m2,m3,m4,m5 = markers_id
	m1 = int(input("Enter tips id:"))
	m2 = int(input("Enter test-tube_1 id:"))
	m3 = int(input("Enter test-tube_2 id:"))
	#m4 = input("Enter trash-can id:")
	#m5 = input("Enter todo id:")
	#print("Pobrane dane: m1: %d, m2: %d, m3: %d" %(m1,m2,m3))
	#print("/aruco_single%d/pose"%(m1))
	m1_sub = rospy.Subscriber("/aruco_single%d/pose"%(m1), PoseStamped, callback0)
	m2_sub = rospy.Subscriber("/aruco_single%d/pose"%(m2), PoseStamped, callback1)
	m3_sub = rospy.Subscriber("/aruco_single%d/pose"%(m3), PoseStamped, callback4)
	#m4_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m4), PoseStamped)
	#m5_sub = message_filters.Subscriber("/aruco_single%d/pose"%(m5), PoseStamped)
	
	#ts = message_filters.TimeSynchronizer([m1_sub,m2_sub,m3_sub], 3) 
	#ts.registerCallback(callback)
	#rospy.spin()
	#sub1 = rospy.Subscriber("/aruco_single%d/pose"%(m1), PoseStamped, callback1)
	#sub2 = rospy.Subscriber("/aruco_single%d/pose"%(m2), PoseStamped, callback2)
	#sub3 = rospy.Subscriber("/aruco_single%d/pose"%(m3), PoseStamped, callback3)
 except:
	print("smth is not yes")

def startPath(move_group):
    
    global markers_ABS_poses
    global matrix_1
    global matrix_2
    global z_orient
    global fill_1_t
    global fill_2_t
    markers_ABS_poses.marker_0 = [grasp_goal[0].position.x, grasp_goal[0].position.y,grasp_goal[0].position.z]
    markers_ABS_poses.marker_1 = [grasp_goal[1].position.x, grasp_goal[1].position.y,grasp_goal[1].position.z]
    markers_ABS_poses.marker_4 = [grasp_goal[2].position.x, grasp_goal[2].position.y,grasp_goal[2].position.z]
    #markers_ABS_poses.marker_5 = [grasp_goal[3].position.x, grasp_goal[3].position.y,grasp_goal[3].position.z]

    #showMarkerPositions()
    
    base_pose = move_group.move_group.get_current_pose().pose
    q_o_1 = [grasp_goal[0].orientation.x, grasp_goal[0].orientation.y, grasp_goal[0].orientation.z, grasp_goal[0].orientation.w]
    q_o_2 = [grasp_goal[1].orientation.x, grasp_goal[1].orientation.y, grasp_goal[1].orientation.z, grasp_goal[1].orientation.w]
    q_o_3 = [grasp_goal[2].orientation.x, grasp_goal[2].orientation.y, grasp_goal[2].orientation.z, grasp_goal[2].orientation.w]

    q_m_1 = euler_from_quaternion(q_o_1)
    q_m_2 = euler_from_quaternion(q_o_2)
    q_m_3 = euler_from_quaternion(q_o_3)

    q_o_n_q_1 = quaternion_from_euler(q_m_1[0]-1.5705,q_m_1[1],q_m_1[2])
    q_o_n_q_2 = quaternion_from_euler(q_m_2[0]-1.5705,q_m_2[1],q_m_2[2])
    q_o_n_q_3 = quaternion_from_euler(q_m_3[0]-1.5705,q_m_3[1],q_m_3[2])

    q_o_n_e_1 = euler_from_quaternion(q_o_n_q_1)
    q_o_n_e_2 = euler_from_quaternion(q_o_n_q_2)
    q_o_n_e_3 = euler_from_quaternion(q_o_n_q_3)

    m1_angle = q_o_n_e_1[2] 
    m2_angle = q_o_n_e_2[2] 
    m3_angle = q_o_n_e_3[2] 
 
    
    
    tip_tower = getTowerPosition(m1_angle, [t_box_ar_1st_t[0], t_box_ar_1st_t[1]])
    water_box_dojazd = getTowerPosition(m2_angle, [w_box_ar_1st_t[0],w_box_ar_1st_t[1]])
    calibrate_tower = getTowerPosition(m3_angle, [cal_box_ar_1st_t[0],cal_box_ar_1st_t[1]])

    matrix_1 = MatrixProb(m2_angle, matrix_1_diameter, matrix_1_size, [grasp_goal[1].position.x+t[0]+water_box_dojazd[0], grasp_goal[1].position.y+t[1]+water_box_dojazd[1],grasp_goal[1].position.z+t[2]], move_group, fill_1_t)

    matrix_2 = MatrixProb(m1_angle, matrix_2_diameter, matrix_2_size, [grasp_goal[0].position.x+t[0]+tip_tower[0], grasp_goal[0].position.y+t[1]+tip_tower[1],grasp_goal[0].position.z+t[2]], move_group,fill_2_t)

    #print("cos: %.3f, sin: %.3f" %(water_box_dojazd[0], water_box_dojazd[1]))
    #print("x: %.3f, y: %.3f, z: %.3f" %(grasp_goal[1].position.x, grasp_goal[1].position.y, grasp_goal[1].position.z))
   
    opt = input("Go to marker id: ")
    while opt != 9:

	if opt == 1:

		z_orient = checkPos(move_group, 1)
    		#plan = move_group.plan_path_to_goal(grasp_goal[1].position.x+t[0]+water_box_dojazd[0], grasp_goal[1].position.y+t[1]+water_box_dojazd[1], grasp_goal[1].position.z+t[2],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
		#move_group.execute_plan(plan)
		#matrixMovement(move_group, m2_angle, diameter_1, matrix_1_size)
		#spiral(move_group,36, 0.01)
		ez = input("Enter prob number to go (1-100): ")
		matrix_1.goToMatrixPoint(ez)

	elif opt == 8:
		
		#if m3_angle < -1.570:
		#	m3_angle-=pi
		#if checkAngle(m3_angle):
		#	z_orient = quaternion_from_euler(0,033,m3_angle,axes="sxyz")
		#else:
		#z_orient = checkPos(move_group, 2)
		#print(z_orient)
		plan = move_group.plan_path_to_goal(grasp_goal[2].position.x+t[0]+calibrate_tower[0], grasp_goal[2].position.y+t[1]+calibrate_tower[1], grasp_goal[2].position.z+t[2],z_orient[0],z_orient[1],z_orient[2],z_orient[3])
		move_group.execute_plan(plan)

	elif opt == 0:
		
		#if checkAngle(m1_angle):
		#	z_orient = quaternion_from_euler(0,0,m1_angle,axes="sxyz")
		#else:
		#	z_orient = quaternion_from_euler(0,0.033,0,axes="sxyz")

		z_orient = checkPos(move_group, 0)
    		#plan = move_group.plan_path_to_goal(grasp_goal[0].position.x+t[0]+tip_tower[0], grasp_goal[0].position.y+t[1]+tip_tower[1], grasp_goal[0].position.z+t[2], z_orient[0], z_orient[1], z_orient[2], z_orient[3])
		#move_group.execute_plan(plan)
		ez = input("Enter prob number to go (1-96): ")
		matrix_2.goToMatrixPoint(ez)
		matrix_2.fillMatrixPoint(ez)
	elif opt == 7:
		z_orient = checkPos(move_group, 0)
		matrix_2.goToMatrixPoint(19)
		matrix_2.fillMatrixPoint(19)
		matrix_1.goToMatrixPoint(4)
		matrix_1.fillMatrixPoint(4)
		matrix_1.goToMatrixPoint(66)
		matrix_1.fillMatrixPoint(66)
		plan = move_group.plan_path_to_goal(grasp_goal[2].position.x+t[0]+calibrate_tower[0], grasp_goal[2].position.y+t[1]+calibrate_tower[1], grasp_goal[2].position.z+t[2]+0.05,z_orient[0],z_orient[1],z_orient[2],z_orient[3])
		move_group.execute_plan(plan)
		
		
		system("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: 1 \nstate: 1.0\"")
		system("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: 1 \nstate: 0.0\"")
		goHome(move_group)
		
		
		
	opt = input("Go to marker id: ")
    
    
    raw_input("enter enter xd")
    #matrixMovement(move_group, (pi+marker1_angle), diameter_1, matrix_1_size, 0.001)
    
    #base_pose = move_group.move_group.get_current_pose().pose
    #plan = move_group.plan_path_to_goal(base_pose.position.x+0.024,
    #base_pose.position.y+0.0263, base_pose.position.z,0, 0,0,1)

    #move_group.execute_plan(plan)

def goDetect(move_group):
	try:
		move_group.go_to_joint_state(90, -145, 90, 20, 90, 1.7)
	except:
		print("smth is not yes")

def goHome(move_group):
	move_group.go_to_joint_state(90, -90, 45, 45, 90, 1.7)
	print("Home position achieved")
	global isHome
	isHome = True

def choseOption(arg, move_group):
	if arg == 0:
		try:
			goDetect(move_group)
		except:
			print("Something went wrong")
	elif arg == 1:
		try:
			goHome(move_group)
			
		except:
			print("Something went wrong")
	elif arg == 2:
		if isHome:
			getMarkerPositions()
		else:
			print("Robot is not in Home position")
	elif arg == 3:
		#try:	
		startPath(move_group)
            	#spiral(move_group, 36, 0.01)
		print("Planned position achieved")
		#except:
		#print("Can not achieve this position")

	elif arg == 4:
			
		#showMarkerPositions()
		matrixMovement(move_group, pi/4.0, diameter_1, matrix_1_size)
	elif arg == 5:
		add_table()
	elif arg == 6:
		system("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: 0 \nstate: 0.0\"")
	elif arg == 7:
		#system("rosservice call /ur_hardware_interface/set_io \"fun: 1 \npin: 0 \nstate: 1.0\"")
		plan = move_group.plan_path_to_goal(grasp_goal[2].position.x+t[0], grasp_goal[2].position.y+t[1], grasp_goal[2].position.z+t[2]+0.05,z_orient[0],z_orient[1],z_orient[2],z_orient[3])
		move_group.execute_plan(plan)
	elif arg == 8:
		global zmienna
		zmienna = 8
	

def main():
  
  try:
    
    while zmienna != 8:
        move_group = MoveGroupPythonInteface()
    	print "Enter which movement you want to execute: "
	print "0. Go to detect position"
    	print "1. Go to home position"
    	print "2. Find AR tag (you need to be in home position)"
    	print "3. Go to detected tag (you need to have detected tag)"
	print "4. Show markers positions"
	print "5. Add cage"
	print "6. Exit program"

	
    	option = input("\nYour choice: ")
    	
    	
    	choseOption(option, move_group)
    
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
