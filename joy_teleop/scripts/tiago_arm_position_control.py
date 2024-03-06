#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
This script is used for teleoperating the Tiago++ robots arms.

Controller mapping:

    /Right_Buttons trackpad -> x, y motion of the mobile base
    /Right_Buttons menu button -> lifts torso by 5 cm
    /Left_Buttons trackpad -> yaw rotation of the mobile base
    /Left_Buttons menu button -> decrease torso height by 5 cm

"""

import sys
import rospy
import math
import numpy as np

import tf
import actionlib

from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Pose
from teleop_tools_msgs.msg import IncrementAction as TTIA
from teleop_tools_msgs.msg import IncrementGoal as TTIG
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import MoveItErrorCodes
# from moveit_python import MoveGroupInterface, PlanningSceneInterface

from trac_ik_python.trac_ik import IK

import moveit_commander

class TiagoArmPositionControl():
    def __init__(self,
                 controller_side='right'):

        ##-------- Variables---------## 
        self.arm_goal_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }

        self.activated = False

        self.controller_side = controller_side
        self.gripper_pressed = False

        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        self.urdf_str = rospy.get_param('/robot_description')

        # self.arm_base_link = "base_footprint"
        self.arm_base_link = "torso_lift_link"
        self.arm_ee_link = "arm_"+ controller_side + "_tool_link"

        self.arm_ik_solver = IK(self.arm_base_link, self.arm_ee_link)

        self.arm_joint_states = [0.0] * self.arm_ik_solver.number_of_joints


        # # -------------- ROS Publishers --------------
        self.target_pose_pub = rospy.Publisher('/arm_'+ controller_side +'_target_pose', Pose, queue_size=1)

        # # -------------- ROS Action Clients --------------
        self.arm_client = actionlib.SimpleActionClient('/arm_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client = actionlib.SimpleActionClient('/parallel_gripper_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.sleep(0.1)

        # # -------------- ROS Subscribers --------------


        rospy.Subscriber('/'+controller_side+'/compensate_pose', PoseStamped, self.__arm_input_callback)

        if controller_side == "right":
            rospy.Subscriber('/Right_Buttons', Joy, self.__gripper_control)

        elif controller_side == "left":
            rospy.Subscriber('/Left_Buttons', Joy, self.__gripper_control)

        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber('/'+controller_side+'/robot_activation', Float64, self.__robot_activation_callback)

        rospy.sleep(0.2)

        rospy.logwarn("\n------------" + self.controller_side + " arm controller initialized------------\n")

    ##-------- Callback Functions---------## 
    def __robot_activation_callback(self, msg):
        # rospy.logwarn("%s", msg.data)

        if msg.data == 2.0 and self.activated == False:
            self.activated = True
            rospy.logwarn(self.controller_side + "arm activated")

        if msg.data == 0.0 and self.activated == True:
            self.activated = False
            rospy.logwarn(self.controller_side + "arm stopped")


    # def __get_arm_left_joint_states(self):
    def __joint_states_callback(self, msg):

        torso_joint = msg.position[20]
        if self.arm_ik_solver.number_of_joints >= 7:
            if self.controller_side == "left":
                self.arm_joint_states = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]] 
            elif self.controller_side == "right":
                self.arm_joint_states = [msg.position[7], msg.position[8], msg.position[9], msg.position[10],
                                          msg.position[11], msg.position[12], msg.position[13]] 

            if self.arm_ik_solver.number_of_joints == 8:
                self.arm_joint_states = self.arm_joint_states.insert(0, torso_joint)

    def __arm_input_callback(self, msg):

        self.arm_goal_pose['position'][0] = msg.pose.position.x
        self.arm_goal_pose['position'][1] = msg.pose.position.y
        self.arm_goal_pose['position'][2] = msg.pose.position.z
        self.arm_goal_pose['orientation'][0] = msg.pose.orientation.x
        self.arm_goal_pose['orientation'][1] = msg.pose.orientation.y
        self.arm_goal_pose['orientation'][2] = msg.pose.orientation.z
        self.arm_goal_pose['orientation'][3] = msg.pose.orientation.w

    def __gripper_control(self, joy_msg):
        if(joy_msg.buttons[1] == 1):
            if not self.gripper_pressed:
                gripper_position = 0.0
                self.gripper_pressed = True
            elif self.gripper_pressed:
                gripper_position = 0.08
                self.gripper_pressed = False

            # Create a trajectory point for the gripper
            gripper_point = JointTrajectoryPoint()
            gripper_point.positions = [gripper_position]
            gripper_point.velocities = []
            gripper_point.accelerations = []
            gripper_point.effort = []
            gripper_point.time_from_start = rospy.Duration(0.5)  # Adjust the duration as needed

            # Create and send the gripper goal
            gripper_goal = FollowJointTrajectoryGoal()
            gripper_goal.trajectory.points = [gripper_point]
            gripper_goal.trajectory.header.stamp = rospy.Time.now()

            gripper_goal.trajectory.joint_names = ["gripper_"+self.controller_side+"_left_finger_joint", "gripper_"+self.controller_side+"_right_finger_joint"]

            self.gripper_client.send_goal(gripper_goal)



    ##-------- Helper Functions---------## 


    def __get_arm_transformation(self):
        try:
            return self.listener.lookupTransform(self.arm_base_link, self.arm_ee_link, rospy.Time(0))
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return None


    def __compose_pose_message(self, target_pose):
        """
        target_pose: dict
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0])
        
        """

        # NOTE: These two checks might not be needed, check function usage.
        if not isinstance(target_pose, dict):
            raise TypeError('target_pose is not a dictionary.')

        for key in ['position', 'orientation']:
            if key not in target_pose:
                raise KeyError(f'key {key} not found in target_pose.')

        pose_message = Pose()
        pose_message.position.x = target_pose['position'][0]
        pose_message.position.y = target_pose['position'][1]
        pose_message.position.z = target_pose['position'][2]

        pose_message.orientation.x = target_pose['orientation'][0]
        pose_message.orientation.y = target_pose['orientation'][1]
        pose_message.orientation.z = target_pose['orientation'][2]
        pose_message.orientation.w = target_pose['orientation'][3]

        return pose_message

    def publish_arm_trajectory(self):
        vel_gain = 0.01  # Adjust the maximum velocity as needed

        if self.activated:  
            # Solve IK
            target_joint_angles = self.arm_ik_solver.get_ik(self.arm_joint_states,
                                                          self.arm_goal_pose['position'][0], self.arm_goal_pose['position'][1], self.arm_goal_pose['position'][2],
                                                          self.arm_goal_pose['orientation'][0], self.arm_goal_pose['orientation'][1], self.arm_goal_pose['orientation'][2], self.arm_goal_pose['orientation'][3])

            self.tf_br.sendTransform((self.arm_goal_pose['position'][0], self.arm_goal_pose['position'][1], self.arm_goal_pose['position'][2]),
                                    (self.arm_goal_pose['orientation'][0], self.arm_goal_pose['orientation'][1], self.arm_goal_pose['orientation'][2], self.arm_goal_pose['orientation'][3]),
                                     rospy.Time.now(), self.controller_side + "_ee_goal", self.arm_base_link)

            # Create a JointTrajectory message
            if target_joint_angles is not None:
                joint_trajectory_goal = FollowJointTrajectoryGoal()
                joint_trajectory_goal.trajectory.header.stamp = rospy.Time.now() # + rospy.Duration(0.3)
                joint_trajectory_goal.trajectory.joint_names = list(self.arm_ik_solver.joint_names)

                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = target_joint_angles
                trajectory_point.velocities = [(end - start) * vel_gain for start, end in zip(self.arm_joint_states, target_joint_angles)]
                trajectory_point.time_from_start = rospy.Duration(0.5)

                joint_trajectory_goal.trajectory.points = [trajectory_point]

                self.arm_client.send_goal(joint_trajectory_goal)
                self.target_pose_pub.publish(self.__compose_pose_message(self.arm_goal_pose))
                # rospy.sleep(0.005)


            else:
                pass
                # rospy.logwarn("unable to solve ik")


    def run(self):
        self._running = True
        while self._running and not rospy.is_shutdown():

            self.publish_arm_trajectory()
            rospy.sleep(0.25)



if __name__ == '__main__':

    try:
        args = rospy.myargv(argv=sys.argv)
        controller_side = args[1]
        rospy.logwarn("controller side: %s", controller_side)
        # controller_side = rospy.get_param(
        # param_name='/controller_side',
        # default='right',
        # )
        rospy.init_node('tiago_arm'+controller_side+'_position_control')
        app = TiagoArmPositionControl(controller_side=controller_side)
        app.run()

    except rospy.ROSInterruptException:
        pass
