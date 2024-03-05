#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

class JoystickTeleop():
    def __init__(self):

        ##-------- Variables---------## 

        self.head_joint_limits = [[-1.24, 1.24], [-0.98, 0.72]]

        self.alpha = 0.7
        self.initialized = False
        self.filtered_orientation = [0.0, 0.0]  # Initial filtered orientation
        self.prev_head_pos = [0.0, 0.0]

        self.activated = False

        self.gripper_pressed = False
        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = rospy.get_param('~forward_rate', 0.8)
        self._angular = 0
        self._linear = 0
        self._linear_y = 0
        self.vel_scale = 0.5
        self.rot_scale = 0.1


        # # -------------- ROS Publishers --------------
        self._pub_cmd = rospy.Publisher('key_vel', Twist, queue_size=10)

        # # -------------- ROS Action Clients --------------
        self.torso_client = actionlib.SimpleActionClient('/torso_controller/increment', TTIA)
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        rospy.sleep(0.1)

        # # -------------- ROS Subscribers --------------

        rospy.Subscriber('/Right_Hand', TransformStamped, self.__right_input_pose_callback)
        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        rospy.Subscriber('/Left_Hand', TransformStamped, self.__left_input_pose_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Head_Motion', PoseStamped, self.__head_motion_callback)
        rospy.Subscriber('/robot_activation', Float64, self.__robot_activation_callback)

        rospy.sleep(0.2)

        rospy.logwarn("\n------------vive teleop ready------------\n\n")

    ##-------- Callback Functions---------## 
    def __robot_activation_callback(self, msg):
        # rospy.logwarn("%s", msg.data)

        if msg.data == 2.0 and self.activated == False:
            self.activated = True
            # rospy.logwarn("robot activated")

        if msg.data == 0.0 and self.activated == True:
            self.activated = False
            # rospy.logwarn("robot stopped")


    # def __get_arm_left_joint_states(self):
    def __joint_states_callback(self, msg):

        torso_joint = msg.position[20]
        if self.arm_right_ik_solver.number_of_joints == 7:
            self.arm_left_joint_states = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]] 

            self.arm_right_joint_states = [msg.position[7], msg.position[8], msg.position[9], msg.position[10],
                                          msg.position[11], msg.position[12], msg.position[13]] 
        elif self.arm_right_ik_solver.number_of_joints == 8:
            self.arm_right_joint_states = [torso_joint, msg.position[7], msg.position[8], msg.position[9], msg.position[10],
                                          msg.position[11], msg.position[12], msg.position[13]] 


    def __right_input_pose_callback(self, msg):

        self.right_controller_x = msg.transform.translation.x
        self.right_controller_y = msg.transform.translation.y
        self.right_controller_z = msg.transform.translation.z
        self.right_controller_rx, self.right_controller_ry, self.right_controller_rz, self.right_controller_rw = msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w

    def __right_input_buttons_callback(self, msg):
        """
            determine if it is squeezing or button press 

        """
        self._base_control(msg)
        if msg.buttons[0] == 1:
            self._torso_increment(0.05)  # Increment for torso up
        self._publish()


    def __left_input_pose_callback(self, msg):

        self.left_controller_x = msg.transform.translation.x
        self.left_controller_y = msg.transform.translation.y
        self.left_controller_z = msg.transform.translation.z
        self.left_controller_rx, self.left_controller_ry, self.left_controller_rz, self.left_controller_rw = msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w

    def __left_input_buttons_callback(self, msg):
        """
            control mobile base using the trackpad on the left controller

        """
        self._base_rotate(msg)

        if msg.buttons[0] == 1:
            self._torso_increment(-0.05)  # Increment for torso up
        self._publish()


    def __head_motion_callback(self, msg):
        """
            control mobile base using the trackpad on the left controller

        """
        if not self.initialized:
            self.initialized = True
            self.filtered_orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]


        # Apply the low-pass filter
        filtered_orientation_x = self.alpha * msg.pose.orientation.x + (1 - self.alpha) * self.filtered_orientation[0]
        filtered_orientation_y = self.alpha * msg.pose.orientation.y + (1 - self.alpha) * self.filtered_orientation[1]
        filtered_orientation_z = self.alpha * msg.pose.orientation.z + (1 - self.alpha) * self.filtered_orientation[2]
        filtered_orientation_w = self.alpha * msg.pose.orientation.w + (1 - self.alpha) * self.filtered_orientation[3]

        roll, pitch, yaw = euler_from_quaternion([filtered_orientation_x, filtered_orientation_y, filtered_orientation_z, filtered_orientation_w])
        # roll, pitch, yaw = euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        self.filtered_orientation = [filtered_orientation_x, filtered_orientation_y, filtered_orientation_z, filtered_orientation_w]


        head_orientation = [round(self.bound(yaw, self.head_joint_limits[1][0], self.head_joint_limits[1][1]), 2),
                            round(self.bound(-pitch, self.head_joint_limits[0][0], self.head_joint_limits[0][1]), 2)]

        # rospy.logwarn("head orientation = %s, %s",round(math.degrees(head_orientation[0])), round(math.degrees(head_orientation[1])))

        self._send_head_goal(head_orientation)
        


    ##-------- Helper Functions---------## 

    def bound(self, low, high, value):
         return max(low, min(high, value))

    def _base_control(self, joy_msg):
        linear = joy_msg.axes[1]
        linear_y = -joy_msg.axes[0]
        self._linear = linear * self._forward_rate if linear > 0 else linear * 0.5
        self._linear_y = linear_y * self._forward_rate if linear > 0 else linear_y * 0.5

    def _base_rotate(self, msg):
        angular = -msg.axes[0]
        self._angular = angular * 1.0


    def _head_control(self, msg):
        # Adjust head position based on joystick axes
        if len(joy_msg.axes) >= 2:
            head_position = [joy_msg.axes[0], joy_msg.axes[1]]
            self._send_head_goal(head_position)

    
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


    def _send_head_goal(self, head_position):

        if not ((abs(self.prev_head_pos[0] - head_position[0]) <= 0.01) and (abs(self.prev_head_pos[1] - head_position[1]) <= 0.01)):

            # Create a trajectory point with the adjusted head position
            point = JointTrajectoryPoint()
            point.positions = head_position
            point.velocities = [abs(self.prev_head_pos[0] - head_position[0]), abs(self.prev_head_pos[1] - head_position[1])]  # Set desired joint velocities
            point.time_from_start = rospy.Duration(0.3)  # A small duration for smooth motion

            # Create and send the head goal
            head_goal = FollowJointTrajectoryGoal()
            head_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
            head_goal.trajectory.points = [point]
            head_goal.trajectory.header.stamp = rospy.Time.now()
            self.head_client.send_goal(head_goal)
        else:
            pass

        self.prev_head_pos = head_position

    def _torso_increment(self, increment_value):
        rospy.loginfo("Torso increment: {}".format(increment_value))
        goal = TTIG()
        goal.increment_by = [increment_value]
        self.torso_client.send_goal(goal)

    def _get_twist(self, linear, linear_y, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = linear_y
        twist.angular.z = angular
        return twist

    def _publish(self):
        twist = self._get_twist(self._linear, self._linear_y, self._angular)
        self._pub_cmd.publish(twist)

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running and not rospy.is_shutdown():
            # rospy.logwarn("%s", self.__get_arm_left_transformation())

            rospy.sleep(0.25)
            # self.plan_and_execute_arm_right_trajectory()
            # rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('joystick_teleop')
        app = JoystickTeleop()
        app.run()
    except rospy.ROSInterruptException:
        pass
