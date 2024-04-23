#!/usr/bin/env python
# -*- coding: utf-8 -*-


"""
This script is used for teleoperating the Tiago++ robots torso, head, and mobile base using gamepad interfaces.

Controller mapping:

    A: base control
    B: right arm control
    X: left arm control
    Y: head control

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
from pal_startup_msgs.srv import StartupStop, StartupStopRequest, StartupStart, StartupStartRequest
from std_msgs.msg import String

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
    def __init__(self, sim=True):

        ## -------------- Variables -------------- ##
        self.sim = sim

        ## ------- Limits ------- ##
        self.head_joint_limits = [[-1.24, 1.24], [-0.98, 0.72]]
        self.torso_limit = 0.07

        ### ------- Head------- ###
        self.alpha = 0.9
        self.filtered_orientation = [0.0, 0.0]  # Initial filtered orientation
        self.prev_head_pos = [0.0, 0.0]
        self.head_vel = 0.1

        ### ------- Torso ------- ###
        self.torso_joint = 0.05

        ### ------- Base ------- ###
        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = rospy.get_param('~forward_rate', 0.15)
        self._angular = 0
        self._linear = 0
        self._linear_y = 0
        self.joy_msg = Joy()


        ## -------------- Flags -------------- ##
        self._mode = 'base'  # Initial mode is base control
        self.arm_side = ''
        self.initialized = False
        self.gripper_pressed = False
        self.activated = False
        self.torso_pressed = False

        ## -------------- ROS Publishers -------------- ##
        if sim:
            self._pub_cmd = rospy.Publisher('key_vel', Twist, queue_size=10)

        else:
            self._pub_cmd = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

        self.pub_arm_left_act = rospy.Publisher('/left/robot_activation', Float64, queue_size=1)
        self.pub_arm_right_act = rospy.Publisher('/right/robot_activation', Float64, queue_size=1)

        ## -------------- ROS Action Clients -------------- ##
        self.torso_client = actionlib.SimpleActionClient('/torso_controller/increment', TTIA)
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_client = [actionlib.SimpleActionClient('/parallel_gripper_left_controller/follow_joint_trajectory', FollowJointTrajectoryAction),
                               actionlib.SimpleActionClient('/parallel_gripper_right_controller/follow_joint_trajectory', FollowJointTrajectoryAction)]


        rospy.sleep(0.1)

        ## -------------- ROS Subscribers -------------- ##
        rospy.Subscriber('/gamepad/joy', Joy, self._joy_callback)


        # rospy.Subscriber('/robot_activation', Float64, self.__robot_activation_callback)
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)

        rospy.sleep(0.2)

        if not sim:

            rospy.wait_for_service('/pal_startup_control/stop')
            try:
                stop_service = rospy.ServiceProxy('/pal_startup_control/stop', StartupStop)
                request = StartupStopRequest()
                request.app = 'head_manager'
                response = stop_service(request)
                # rospy.logwarn("%s", request)
                # rospy.logwarn("%s", response)
            except rospy.ServiceException as e:
                print("Service call failed:", e)


        rospy.logwarn("\n\n------------gamepad teleop ready------------\n\n")



    ## -------------- Callback Functions -------------- 

    def _joy_callback(self, joy_msg):

        # Change modes (A: [0] -> base / B: [1] -> right arm / X: [2] -> left arm / Y: [3] -> head)
        if joy_msg.buttons[0] and not self._mode == 'base':
            rospy.logwarn("changed to base mode")
            self._mode = 'base'

        elif joy_msg.buttons[1] and not self._mode == 'right_arm':
            rospy.logwarn("changed to right arm mode")
            self._mode = 'right_arm'

        elif joy_msg.buttons[2] and not self._mode == 'left_arm':
            rospy.logwarn("changed to left arm mode")
            self._mode = 'left_arm'

        elif joy_msg.buttons[3] and not self._mode == 'head':
            rospy.logwarn("changed to head mode")
            self._mode = 'head'

        self.joy_msg = joy_msg
        self.initialized = True

    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        

    ## -------------- Helper Functions -------------- 

    def bound(self, low, high, value):
         return max(low, min(high, value))

    def _base_control(self, joy_msg):

        linear = joy_msg.axes[1]
        linear_y = joy_msg.axes[0]

        # filter out joystick noise
        if(abs(linear) > 0.1 or abs(linear_y) > 0.1):
            self._linear = linear * self._forward_rate 
            self._linear_y = linear_y * self._forward_rate 
        else:
            self._linear = 0
            self._linear_y = 0

    def _base_rotate(self, msg):
        angular = msg.axes[3]
        if(abs(angular) > 0.1):
            self._angular = angular * 1.0
        else:
            self._angular = 0.0


    def _head_control(self, joy_msg):
        # Adjust head position based on joystick axes
        if(abs(joy_msg.axes[0]) > 0.1 or abs(joy_msg.axes[1]) > 0.1):
            head_position = [joy_msg.axes[0], joy_msg.axes[1]]

            head_goal = [self.prev_head_pos[0] + self.head_vel * head_position[0], self.prev_head_pos[1] + self.head_vel * head_position[1]]

            head_orientation = [round(self.bound(head_goal[0], self.head_joint_limits[0][0], self.head_joint_limits[0][1]), 2),
                            round(self.bound(head_goal[1], self.head_joint_limits[1][0], self.head_joint_limits[1][1]), 2)]


            self._send_head_goal(head_goal)

    
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

        # Create a trajectory point with the adjusted head position
        point = JointTrajectoryPoint()
        point.positions = head_position
        point.velocities = [abs(self.prev_head_pos[0] - head_position[0]), abs(self.prev_head_pos[1] - head_position[1])]  # Set desired joint velocities
        point.time_from_start = rospy.Duration(0.4)  # A small duration for smooth motion

        # Create and send the head goal
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        head_goal.trajectory.points = [point]
        head_goal.trajectory.header.stamp = rospy.Time.now()
        self.head_client.send_goal(head_goal)

        # Update prev_head_pos
        self.prev_head_pos = head_position

    def _torso_increment(self, increment_value):
        goal = TTIG()
        goal.increment_by = [increment_value]
        if self.torso_joint >= self.torso_limit:

            self.torso_client.send_goal(goal)
            self.torso_client.wait_for_result()
        else:
            rospy.logwarn("Torso: lower limit")
            if increment_value > 0.0:
                self.torso_client.send_goal(goal)

    def __gripper_control(self, joy_msg):
        ## if trigger pressed
        if(joy_msg.axes[2] == -1 or joy_msg.axes[5] == -1):
            if not self.trigger_pressed:  # If trigger was not pressed before
                if not self.gripper_pressed:  # If gripper is currently closed
                    gripper_position = 0.0  # Open the gripper
                    self.gripper_pressed = True
                else:
                    gripper_position = 0.08  # Close the gripper
                    self.gripper_pressed = False

                
                side = 'left' if joy_msg.axes[2] == -1 else 'right'
                idx = 0 if joy_msg.axes[2] == -1 else 1


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

                gripper_goal.trajectory.joint_names = ["gripper_"+side+"_left_finger_joint", "gripper_"+side+"_right_finger_joint"]

                self.gripper_client[idx].send_goal(gripper_goal)

            self.trigger_pressed = True  # Update trigger state
        else:
            self.trigger_pressed = False  # Update trigger state



    def _get_twist(self, linear, linear_y, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.linear.y = linear_y
        twist.angular.z = angular
        return twist

    def _publish(self):

        twist = self._get_twist(self._linear, self._linear_y, self._angular)
        self._pub_cmd.publish(twist)


    def _control(self):
        if self.initialized:
            # Check current mode and execute corresponding control
            if self._mode == 'base':
                self.pub_arm_left_act.publish(0.0)
                self.pub_arm_right_act.publish(0.0)
                
                self._base_control(self.joy_msg)
                self._base_rotate(self.joy_msg)

                if abs(self.joy_msg.axes[7]) == 1.0 and not self.torso_pressed:
                    self.torso_pressed = True
                    self._torso_increment(np.sign(self.joy_msg.axes[7]) * 0.05)
                    rospy.sleep(0.2)
                elif self.joy_msg.axes[7] == 0.0:
                    self.torso_pressed = False

            elif self._mode == 'head':
                self.pub_arm_left_act.publish(0.0)
                self.pub_arm_right_act.publish(0.0)

                self._head_control(self.joy_msg)


            elif self._mode == 'left_arm':

                self.pub_arm_left_act.publish(2.0)
                self.pub_arm_right_act.publish(0.0)

            elif self._mode == 'right_arm':

                self.pub_arm_left_act.publish(0.0)
                self.pub_arm_right_act.publish(2.0)

            self.__gripper_control(self.joy_msg)


    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running and not rospy.is_shutdown():
            # rospy.logwarn("%s", self.__get_arm_left_transformation())
            self._publish()
            self._control()
            # rospy.sleep(0.25)
            # self.plan_and_execute_arm_right_trajectory()
            rate.sleep()

def node_shutdown():
    """
    
    """
    rospy.wait_for_service('/pal_startup_control/start')
    try:
        start_service = rospy.ServiceProxy('/pal_startup_control/start', StartupStart)
        request = StartupStartRequest()
        request.app = 'head_manager'
        response = start_service(request)
        # rospy.logwarn("%s", request)
        # rospy.logwarn("%s", response)

        print('\nhead_manager started back up\n')
    except rospy.ServiceException as e:
        print('\nFailed to start head_manager\n')



def node_shutdown_sim():
    print('\nvive_teleop has been shutdown\n')


def main():
    try:
        rospy.init_node('vive_teleop')

        args = rospy.myargv(argv=sys.argv)
        sim = args[1]

        if sim == "true":
            sim_param = True
            rospy.on_shutdown(node_shutdown_sim)
        elif sim == "false":
            sim_param = False
            rospy.on_shutdown(node_shutdown)

        app = JoystickTeleop(sim=sim_param)
        app.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

