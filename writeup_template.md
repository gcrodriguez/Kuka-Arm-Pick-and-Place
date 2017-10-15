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

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


