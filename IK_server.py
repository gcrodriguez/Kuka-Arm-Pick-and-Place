#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya 

# import modules
import rospy, tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
	### Your FK code here
        # Create symbols
	q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #joint angles
	d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #links offsets
	a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #links lenghts
	alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') #rotation angles about X axis
   
	# Create Modified DH parameters
	s = {alpha0:	 0, a0:	     0, d1: 0.75,	
	     alpha1: -pi/2, a1:   0.35, d2:    0,   q2: q2 - pi/2,	
	     alpha2:     0, a2:   1.25, d3:    0,
             alpha3: -pi/2, a3: -0.054, d4:  1.501,
	     alpha4:  pi/2, a4:      0, d5:    0,
	     alpha5: -pi/2, a5:      0, d6:    0,
	     alpha6:     0, a6:      0, d7: 0.303,  q7: 0}  #dictionary DH values for Kuka KR210
	
	# Define Modified DH Transformation matrix and individual transformation matrices
	T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
               [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
               [                   0,                   0,            0,               1]])
	T0_1 = T0_1.subs(s)

	T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
               [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
               [                   0,                   0,            0,               1]])
	T1_2 = T1_2.subs(s)

	T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
               [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
               [                   0,                   0,            0,               1]])
	T2_3 = T2_3.subs(s)

	T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
               [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
               [                   0,                   0,            0,               1]])
	T3_4 = T3_4.subs(s)

	T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
               [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
               [                   0,                   0,            0,               1]])
	T4_5 = T4_5.subs(s)


	T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
               [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
               [                   0,                   0,            0,               1]])
	T5_6 = T5_6.subs(s)

	T6_G = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
               [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
               [                   0,                   0,            0,               1]])
	T6_G = T6_G.subs(s)

	T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G #base link to gripper
	
	# Extract rotation matrices from the transformation matrices
	R0_1 = T0_1[0:-1,0:-1]
	R1_2 = T1_2[0:-1,0:-1]
	R2_3 = T2_3[0:-1,0:-1]
	R3_4 = T3_4[0:-1,0:-1]
	R4_5 = T4_5[0:-1,0:-1]
	R5_6 = T5_6[0:-1,0:-1]
	R6_G = T6_G[0:-1,0:-1]
	R0_3 = simplify(R0_1 * R1_2 * R2_3)
	
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	    # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            ### Your IK code here 
	    # Compensate for rotation discrepancy between DH parameters and Gazebo
	    roll1, pitch1, yaw1 = symbols('roll1 pitch1 yaw1')

	    R_z = Matrix([[ cos(yaw1), -sin(yaw1),  0,  0],
              		  [ sin(yaw1),  cos(yaw1),  0,  0],
                          [ 0,              0,      1,  0],
			  [ 0,		    0,      0,  1]])

	    R_y = Matrix([[ cos(pitch1),        0,    sin(pitch1),  0],
              		  [       0,            1,        0,        0],
                          [-sin(pitch1),        0,    cos(pitch1),  0],
			  [       0,            0,        0,        1]])

	    R_x = Matrix([[ 1, 	   0,           0,        0],
              		  [ 0,  cos(roll1), -sin(roll1),  0],
                          [ 0,  sin(roll1),  cos(roll1),  0],
			  [ 0,      0,          0,        1]])

	    R_corr = R_z.subs(yaw1, radians(180)) * R_y.subs(pitch1, radians(-90))

	    ## Total Transformation Matrix corrected
	    T_total = T0_G * R_corr

	    # Calculate joint angles using Geometric IK method
	    ## Wrist center position
	    Rrpy = R_z * R_y * R_x * R_corr
	    Rrpy = Rrpy.subs({'roll1': roll, 'pitch1': pitch, 'yaw1': yaw}) #transformation matrix with 											angles obtained in ROS
            pg= Matrix([[px], [py], [pz]])
            Wc = pg - 0.303*Rrpy[:-1, 2]

            ## Constants
            a1 = 0.35
            a2 = 1.25
            a3 = -(-0.054)   #must have negative value as it is being measured in relation of base link 				       (a3 originally is negative because fram 4 is below frame 3)
            d1 = 0.75
            d4 = 1.5
            L3 = sqrt(d4*d4 + a3*a3)
            L2 = a2

            ##Finding theta2
            d11 = sqrt((Wc[0] * Wc[0]) + (Wc[1] * Wc[1])) - a1
            d12 = Wc[2] - d1
            d13 = sqrt((d11 * d11) + (d12 * d12))
            ro = atan2(d12, d11)
            phi = acos(-(L3 * L3 - L2 * L2 - d13 * d13)/(2 * L2 * d13))
            theta2 = radians(90) - phi - ro

            ##Finding theta3
            epslon = atan(a3/d4)
            psi = acos(-(d13 * d13 - L2 * L2 - L3 * L3)/(2 * L2 * L3))
            theta3 = radians(90) - psi - epslon

            ##Finding theta1
            theta1 = atan2(Wc[1], Wc[0])

            #Orientation matrix
            R3_G = R0_3.inv("LU") * Rrpy[0:-1, 0:-1]
            Rot = R3_G.subs({'q1': theta1, 'q2': theta2, 'q3': theta3})

            ##Finding theta4
            theta4 = atan2(Rot[2, 2], -Rot[0, 2])

            ##Finding theta5
            theta5 = atan2(sqrt(Rot[1, 0] * Rot[1, 0] + Rot[1, 1] * Rot[1, 1]), Rot[1, 2])

            ##Finding theta6
            theta6 = atan2(-Rot[1, 1], Rot[1, 0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
