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
from numpy import linalg as LA
import time
import tf
import actionlib
import transformations
from scipy.interpolate import CubicSpline

from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
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
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult


# from moveit_python import MoveGroupInterface, PlanningSceneInterface

from trac_ik_python.trac_ik import IK

import moveit_commander

class TiagoArmPositionControl():
    def __init__(self,
                 controller_side='right',
                 sim=True):

        ##-------- Variables---------## 
        self.controller_side = controller_side

        self.translation_rate = 0.001
        self.rotation_rate = 0.003
        self.loop_rate = 0.005

        self.listener = tf.TransformListener()
        self.tf_br = tf.TransformBroadcaster()

        ## ------- Limits ------- ##
        self.rotation_limits = [[-1.57, -1.57, -1.57], [1.57, 1.57, 1.57]]

        ### ------- Arm ------- ###

        self.urdf_str = rospy.get_param('/robot_description')
        # self.arm_base_link = "base_footprint"
        self.arm_base_link = "torso_lift_link"
        self.arm_ee_link = "arm_"+ controller_side + "_tool_link"
        self.arm_ik_solver = IK(self.arm_base_link, self.arm_ee_link)
        self.arm_joint_states = [0.0] * self.arm_ik_solver.number_of_joints

        current_ee  = self.__get_arm_transformation()
        if current_ee is not None:
            self.arm_goal_pose = {
                'position': np.array([current_ee.position.x, current_ee.position.y, current_ee.position.z]),
                'orientation': np.array([current_ee.orientation.x, current_ee.orientation.y, current_ee.orientation.z, current_ee.orientation.w]),
            }
        else:
            self.arm_goal_pose = {
                'position': np.array([0.0, 0.0, 0.0]),
                'orientation': np.array([0.0, 0.0, 0.0, 1.0]),
            }

        self.delta_x, self.delta_y, self.delta_z, self.delta_roll, self.delta_pitch, self.delta_yaw = 0, 0, 0, 0, 0, 0

        ### ------- Torso ------- ###
        self.torso_joint = 0.2

        ## -------------- Flags -------------- ##
        self.homing = False
        self.trigger_pressed = False
        self.activated = False
        self.gripper_pressed = False



        ## -------------- ROS Publishers --------------
        self.target_pose_pub = rospy.Publisher('/arm_'+ controller_side +'_target_pose', Pose, queue_size=1)

        ## -------------- ROS Action Clients -------------- ##
        if sim:
            self.arm_client = actionlib.SimpleActionClient('/arm_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)


        else:
            self.arm_client = actionlib.SimpleActionClient('/safe_arm_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.arm_traj_pub = rospy.Publisher('/arm_'+ controller_side +'_controller/safe_command', JointTrajectory, queue_size=1)

        self.gripper_client = actionlib.SimpleActionClient('/parallel_gripper_'+controller_side+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.play_motion_client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)

        rospy.sleep(0.1)

        ## -------------- ROS Subscribers --------------
        rospy.Subscriber('/'+controller_side+'/compensate_pose', PoseStamped, self.__arm_input_callback)

        rospy.Subscriber('/gamepad/joy', Joy, self._joy_callback)
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber('/'+controller_side+'/robot_activation', Float64, self.__robot_activation_callback)

        rospy.sleep(0.2)


    ##-------- Callback Functions---------## 
    def __robot_activation_callback(self, msg):
        # rospy.logwarn("%s", msg.data)

        if msg.data == 2.0 and self.activated == False and not self.homing:
            self.activated = True
            current_ee  = self.__get_arm_transformation()
            self.arm_goal_pose = {
                'position': np.array([current_ee.position.x, current_ee.position.y, current_ee.position.z]),
                'orientation': np.array([current_ee.orientation.x, current_ee.orientation.y, current_ee.orientation.z, current_ee.orientation.w]),
            }
            rospy.logwarn(self.controller_side + "arm activated")

        if msg.data == 0.0 and self.activated == True and not self.homing:
            self.activated = False
            rospy.logwarn(self.controller_side + "arm stopped")


    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        if self.arm_ik_solver.number_of_joints >= 7:
            if self.controller_side == "left":
                self.arm_joint_states = [msg.position[0], msg.position[1], msg.position[2], msg.position[3], msg.position[4], msg.position[5], msg.position[6]] 

            elif self.controller_side == "right":
                self.arm_joint_states = [msg.position[7], msg.position[8], msg.position[9], msg.position[10],
                                          msg.position[11], msg.position[12], msg.position[13]] 

            if self.arm_ik_solver.number_of_joints == 8:
                self.arm_joint_states = self.arm_joint_states.insert(0, torso_joint)

    def __arm_input_callback(self, msg):
        # self.prev_arm_goal_pose = copy.deepcopy(self.arm_goal_pose)
        self.arm_goal_pose['position'][0] = msg.pose.position.x
        self.arm_goal_pose['position'][1] = msg.pose.position.y
        self.arm_goal_pose['position'][2] = msg.pose.position.z
        self.arm_goal_pose['orientation'][0] = msg.pose.orientation.x
        self.arm_goal_pose['orientation'][1] = msg.pose.orientation.y
        self.arm_goal_pose['orientation'][2] = msg.pose.orientation.z
        self.arm_goal_pose['orientation'][3] = msg.pose.orientation.w

    def _joy_callback(self, msg):
        """
        update arm end-effector goal based on joystick input

        """
        self.delta_x = self.translation_rate * msg.axes[1] if abs(msg.axes[1]) > 0.1 else 0
        self.delta_y = self.translation_rate * msg.axes[0] if abs(msg.axes[0]) > 0.1 else 0
        self.delta_z = self.translation_rate * msg.axes[4] if abs(msg.axes[4]) > 0.1 else 0
        self.delta_roll = self.rotation_rate * round(msg.axes[6], 2) if abs(msg.axes[6]) > 0.1 else 0
        self.delta_pitch = self.rotation_rate * round(msg.axes[7], 2) if abs(msg.axes[7]) > 0.1 else 0
        self.delta_yaw = self.rotation_rate * round(msg.axes[3], 2) if abs(msg.axes[3]) > 0.1 else 0

        if msg.buttons[7] == 1.0 and self.activated and not self.homing_pressed:
            self.homing_pressed = True
            self.arm_homing()
        if msg.buttons[7] == 0.0:
            self.homing_pressed = False


    ##-------- Helper Functions---------## 


    def __get_arm_transformation(self):
        """
        get arm end-effector tf transform w.r.t. arm base

        """

        start_time = time.time()
        while time.time() - start_time < 5:  # Look for transform for 5 seconds
            try:
                (trans, rot) = self.listener.lookupTransform(self.arm_base_link, self.arm_ee_link, rospy.Time(0))
                pose_message = Pose()
                pose_message.position.x = trans[0]
                pose_message.position.y = trans[1]
                pose_message.position.z = trans[2]

                pose_message.orientation.x = rot[0]
                pose_message.orientation.y = rot[1]
                pose_message.orientation.z = rot[2]
                pose_message.orientation.w = rot[3]

                return pose_message

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform, retrying...")
                rospy.sleep(0.1)  # Adjust the sleep duration as needed
                continue


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
        vel_gain = 1  # Adjust the maximum velocity as needed

        if self.activated:  
            # Solve IK
            if self.arm_joint_states is not None:
                t = time.time()

                target_joint_angles = self.arm_ik_solver.get_ik(self.arm_joint_states,
                                                          self.arm_goal_pose['position'][0], self.arm_goal_pose['position'][1], self.arm_goal_pose['position'][2],
                                                          self.arm_goal_pose['orientation'][0], self.arm_goal_pose['orientation'][1], self.arm_goal_pose['orientation'][2], self.arm_goal_pose['orientation'][3])

                self.tf_br.sendTransform((self.arm_goal_pose['position'][0], self.arm_goal_pose['position'][1], self.arm_goal_pose['position'][2]),
                                        (self.arm_goal_pose['orientation'][0], self.arm_goal_pose['orientation'][1], self.arm_goal_pose['orientation'][2], self.arm_goal_pose['orientation'][3]),
                                         rospy.Time.now(), self.controller_side + "_ee_goal", self.arm_base_link)


                # # Create a JointTrajectory message
                if target_joint_angles is not None:
                    if LA.norm(np.array(target_joint_angles) - np.array(self.arm_joint_states)) < np.pi/4:

                        joint_trajectory_goal = JointTrajectory()

                        joint_trajectory_goal.header.stamp = rospy.Time.now() # + rospy.Duration(0.3)
                        joint_trajectory_goal.joint_names = list(self.arm_ik_solver.joint_names)

                        trajectory_point = JointTrajectoryPoint()
                        trajectory_point.positions = target_joint_angles
                        trajectory_point.velocities = [(end - start) * vel_gain for start, end in zip(self.arm_joint_states, target_joint_angles)]
                        trajectory_point.time_from_start = rospy.Duration(0.5)

                        joint_trajectory_goal.points.append(trajectory_point)

                        # self.arm_client.send_goal(joint_trajectory_goal)
                        self.arm_traj_pub.publish(joint_trajectory_goal)


                        self.target_pose_pub.publish(self.__compose_pose_message(self.arm_goal_pose))
                    else:
                        # rospy.logwarn("large norm: %s", LA.norm(np.array(target_joint_angles) - np.array(self.arm_joint_states)))
                        pass


                else:
                    pass
                    # rospy.logwarn("unable to solve ik")

    def update_arm_ee(self):
        if self.activated:
            # if abs(self.delta_x) > 0.01 or abs(self.delta_y) > 0.01 or abs(self.delta_z) > 0.01: 
            self.arm_goal_pose['position'][0] = self.arm_goal_pose['position'][0] + self.delta_x
            self.arm_goal_pose['position'][1] = self.arm_goal_pose['position'][1] + self.delta_y
            self.arm_goal_pose['position'][2] = self.arm_goal_pose['position'][2] + self.delta_z


            current_ee  = self.__get_arm_transformation()
            current_ee_pos = [current_ee.position.x, current_ee.position.y, current_ee.position.z]
            if current_ee is not None and LA.norm(np.array(self.arm_goal_pose['position']) - np.array(current_ee_pos)) > 0.2:
                self.arm_goal_pose = {
                    'position': np.array([current_ee.position.x, current_ee.position.y, current_ee.position.z]),
                    'orientation': np.array([current_ee.orientation.x, current_ee.orientation.y, current_ee.orientation.z, current_ee.orientation.w]),
                }
            # if abs(self.delta_roll) > 0.01 or abs(self.delta_pitch) > 0.01 or abs(self.delta_yaw) > 0.01: 

            roll, pitch, yaw = euler_from_quaternion(self.arm_goal_pose['orientation'])
            delta_euler = [roll + self.delta_roll, pitch + self.delta_pitch, yaw + self.delta_yaw]
            rect_delta_euler = np.clip(np.array(delta_euler), self.rotation_limits[0], self.rotation_limits[1])
            delta_orientation = quaternion_from_euler(rect_delta_euler[0], rect_delta_euler[1], rect_delta_euler[2])

            
            self.arm_goal_pose['orientation'] = delta_orientation



    def arm_homing(self):
        ## stop teleop during arm homing
        self.activated = False
        self.homing = True

        rospy.logwarn("moving to home position...")
        motion_name = "tele_home_" + self.controller_side + "_v2"
        home_goal = PlayMotionGoal(motion_name = motion_name, skip_planning = False)
        self.play_motion_client.send_goal(home_goal)
        self.play_motion_client.wait_for_result()

        arm_home_pose = self.__get_arm_transformation()
        rospy.sleep(0.2)
        self.target_pose_pub.publish(arm_home_pose)
        self.tf_br.sendTransform((arm_home_pose.position.x, arm_home_pose.position.y, arm_home_pose.position.z),
                                (arm_home_pose.orientation.x, arm_home_pose.orientation.y, arm_home_pose.orientation.z, arm_home_pose.orientation.w),
                                 rospy.Time.now(), self.controller_side + "_ee_goal", self.arm_base_link)
        current_ee  = self.__get_arm_transformation()
        self.arm_goal_pose = {
            'position': np.array([current_ee.position.x, current_ee.position.y, current_ee.position.z]),
            'orientation': np.array([current_ee.orientation.x, current_ee.orientation.y, current_ee.orientation.z, current_ee.orientation.w]),
        }
        rospy.logwarn("moved to home")
        rospy.sleep(0.2)

        self.homing = False
        self.activated = True


    def run(self):
        self._running = True
        while self._running and not rospy.is_shutdown():
            self.update_arm_ee()
            self.publish_arm_trajectory()
            rospy.sleep(self.loop_rate)


## -------------- Node Destructor -------------- ## 

def node_shutdown():
    print(f'\nArm position control has been shut down...\n')


## -------------- Main Function -------------- ## 

def main():
    try:
        args = rospy.myargv(argv=sys.argv)
        controller_side = args[1]
        sim = args[2]
        if sim == "true":
            sim_param=True
        elif sim == "false":
            sim_param = False
        rospy.init_node('tiago_arm'+controller_side+'_position_control')
        app = TiagoArmPositionControl(controller_side=controller_side, sim=sim_param)
        app.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

