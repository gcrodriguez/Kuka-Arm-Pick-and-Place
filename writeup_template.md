## Project: Kinematics Pick & Place
### This writeup contains the methods used for the pick & place project. Basically it describes how the forward and inverse kinematics of the Kuka Arm were defined and implemented in order to pick the object from the shelf and thrown it in the bin.

---
![image1](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/misc2.png)

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Below there is an example of one of the kuka arm pose. This mode was important for checking the codes of direct and inverse kinematics, as well as the dimensions of the robot arm.

![image2](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/Kuka_FK.png)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

The table below shows the DH parameter used in the code. As the joint theta angles (also represented by q's) are the variables of interest, the shall not be listed in the table.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | 
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  - pi/2 | -0.054 | 1.501 | 0
4->5 |   pi/2 | 0 | 0 | 0
5->6 | - pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

The transformation matrices about each joint were obtained using the matrix as defined in class (image below). In the following image, it is represented an example of the transformation matrix corresponding to the gripper in relation of link 6 (T6_G).

![image3](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/dh-transform-matrix.png)

![image4](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/T0_6_code.png)

Finally, to obtain the final transformation matrix corresponding to the gripper in relation of the base, individual transformation matrices about each joint shall be multiplied in the sequence as demonstrated below.

T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

Before proceeding, as mentioned in class, the correction matrix (R_corr) shall multiply T0_G in order to correlate the orientation which the final transformation matrix (using DH parameters) describes and what is being seen in the robot's pose (urdf file).

The correction matrix is composed by rotations about 180 deg in the Z axis and -90 deg in the Y axis, as can be observed in the image below of the implementation in the code.

![image5](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/correction.png)

Therefore, through the final matrix it is possible to extract the final orientations, important for use in inverse kinematics (to define wrist angles). Moreover, the position error between the final position of the gripper defined by the inverse kinematics and point defined in the trajectory planner (using the code IK_debug.py).

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

To solve de inverse kinematics problem it has to be decoupled in inverse position kinematics and inverse orientation kinematics. Solving the inverse position kinematics will provide the angles theta 1, 2 and 3. As mentioned in class, trigonometry is the best ally to deal with this situation. Then separating the robot arm up to the wrist center, the image below describes the trigonometry scenario considered to find the firs three joint angles.

![image6](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK.png)

It is important to notice that:
 - Epslon angle is fixed and defined by the atan(a3/d4). At initial position, if epslon was equal to zero, L3 would be orthogonal to L2.
 - At initial position, when theta 2 equal zero, L2 is orthogonal to a1.
 - Carrefull must be taken when finding d13. It is not simply the diference between Wcx and a1. As the robot arm rotates about Z0 axis,    a1, which is fixed, rotates and it makes difference for d13 and d11.
 - The position (px,py, pz) and orientation (roll, pitch, yaw) of the gripper (end effector) will be inputs for the inverse kinematics.    The trajectory planner will provide these parameters to the IK_server through a service call.

The first step that must be taken is to find the wirst center position in relation to the base. It can be obtained by subtracting form the gripper position (px,py, pz) the wirst length rotated by the wirst angles (roll, pitch, yaw). Then:

     [[Wcx], [Wcy], [wcz]] = ([[px], [py], [pz]] - d7*Rrpy[:-1, 2], where Rrpy[:-1, 2] is obtained from Rrpy = R_z * R_y * R_x * R_corr 

Observing the image and based on the considerations below, it can be concluded that:

     theta2 + phi + ro = 90deg ==> theta2 = 90deg - phi - ro

     psi + theta3 + epslon = 90deg ==> theta3 = 90deg - psi - epslon

The goal now is to obtain phi, ro and psi. The ro angle is simply achieved by observing the d11d12d13 triangle:

     ro = atan2(d12/d11) , where : d12 = Wcz - d1 and d11 = sqrt(Wcy² + Wcx²) - a1
     
One way to quick obtain phi and psi angles is to do two times the cosine law by observing the d13L2L3 triangle;

     phi = acos(-(L3² - L2² - d13²)/(2*L2*d13)), where L2 = a2, L3 = sqrt(a3² + d4²) and d13 = sqrt(d11² + d12²)
     
     psi = acos(-(d13² - L2² - L3²)/(2*L2*L3))

 Finally, theta1 is obtained by the (a1 + d11)WcxWcy triangle showed in the top view:
     
     theta1 = atan2(Wcy/Wcx)

Now, with the relations of thetas 1, 2 and 3 stablished, it will be useful to find thetas 4, 5 and 6 by solving the inverse orientation kinematics. The trick here is to manipulate the rotation matrices, as the angles of the gripper in relation to the base is an input. Then Rrpy matrix which is R0_G can be obtained by applying roll, pitch and yaw angles, as made to find Wirst position.

     R0_G = Rrpy(function of roll, pitch , yaw)
  
But R0_G = R0_3 * R3_G ===> R0_3 * R3_G = Rrpy(function of roll, pitch , yaw)

Then:
     R3_G = inv(R0_3) * R3_G,  where R0_3 is the rotation matriz of the wirst center in relation to the base using thetas 1, 2 and 3                                    obtained in the inverser position kinematics
     
 And the matriz of the left side:

       R3_G = Matrix([[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)]
                      [sin(q5)*cos(q6),                            -sin(q5)*sin(q6),                                    cos(q5)]
                      [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])

By comparing the matricez of the right side (above) with the left side containing values, the relations for thetas 4, 5 and 6 were obtained, as specified below:

       Rrpy[2,2] / -Rrpy[0,2] = sin(q4)*sin(q5)/-(-sin(q5)*cos(q4)) ==> theta4 = atan2(Rrpy[2,2] / Rrpy[0,2])

       sqrt(Rrpy[1,0]² + Rrpy[1,1]²) / Rrpy[1,2] = sqrt((sin(q5)*cos(q6)² + (-sin(q5)*sin(q6))²)/cos(q5)
          ==> theta5 = atan2(sqrt(Rrpy[1,0]² + Rrpy[1,1]²) / Rrpy[1,2]
          
 NOTE: For theta formulation, sqrt(Rrpy[1,0]² + Rrpy[1,1]²) could be exchanged by sqrt(Rrpy[0,2]² + Rrpy[2,2)
 
       -Rrpy[1,1] / Rrpy[1,0] = -(-sin(q5)*sin(q6))/sin(q5)*cos(q6) ==> theta6 = atan2(-Rrpy[1,1] / Rrpy[1,0])
       
       
With all relations described above the joint angles can be provided to the simulation when requested by the service call with an kuka's arm pose.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The results obtained in the IK_debug for three cases showed that error in the gripper position is below than 0.00000000 for the tree cases simulated, as shown in the images below. However, the total run time to calculate one pose is about 70 sec which makes a very long simulation in the gazebo world.

 - Simulation 1
 
![image7](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK_debug_1.png)

 - Simulation 2
 
![image8](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK_debug_2.png)

- Simulation 3
 
![image9](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK_debug_3.png)

The code used to run the simulations above used simplify matrices multplication. With this all really smal components were removed. But when the simplify command was removed from the code, the total run time to calculate one pose decreases to about sec. However, the position error increased, as can be observed below. 

 - Simulation 1
 
![image10](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK_debug_1_faster.png)

 - Simulation 2
 
![image11](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK_debug_2_faster.png)

- Simulation 3
 
![image12](https://github.com/gcrodriguez/Kuka-Arm-Pick-and-Place/blob/master/IK_debug_3_faster.png)


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


