
"""
This script is used for recording the robot states, base trajectory, base to target, arms to target, and controller states.


Author: Juo-Tung Chen


"""

import csv
import os 
import datetime
import rospy
from geometry_msgs.msg import Pose, Twist, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64, Bool

import tf
import sys
import rospy
import numpy as np
## -------------- Class Definition -------------- ##


class DataRecorder():
    def __init__(self):

        ## -------------- Files -------------- ##
        timestamp = datetime.datetime.now().strftime("%m%d_%H%M%S")
        self.robot_states_filename = os.path.expanduser(f"~/RAL2024/teleop_data/robot_states/{timestamp}.csv")
        self.base_trajectory_filename = os.path.expanduser(f"~/RAL2024/teleop_data/robot_trajectory/{timestamp}.csv")
        self.base_to_target_filename = os.path.expanduser(f"~/RAL2024/teleop_data/robot_to_target/{timestamp}.csv")
        self.arm_left_to_target_filename = os.path.expanduser(f"~/RAL2024/teleop_data/arm_left_to_target/{timestamp}.csv")
        self.arm_right_to_target_filename = os.path.expanduser(f"~/RAL2024/teleop_data/arm_right_to_target/{timestamp}.csv")
        self.controller_states_filename = os.path.expanduser(f"~/RAL2024/teleop_data/controller_states/{timestamp}.csv")


        ### ---------- Ensure the directory exists --------- ###
        self.__make_sure_path_exists(self.robot_states_filename)
        self.__make_sure_path_exists(self.base_trajectory_filename)
        self.__make_sure_path_exists(self.base_to_target_filename)
        self.__make_sure_path_exists(self.arm_left_to_target_filename)
        self.__make_sure_path_exists(self.arm_right_to_target_filename)
        self.__make_sure_path_exists(self.controller_states_filename)

        ### -------------- Write Headers -------------- ###
        with open(self.robot_states_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "left_arm_state", "left_arm_extended", "left_arm_x", "left_arm_y", "left_arm_z", "left_arm_qx", "left_arm_qy", "left_arm_qz", "left_arm_qw",
                             "right_arm_state","right_arm_extended", "right_arm_x", "right_arm_y", "right_arm_z", "right_arm_qx", "right_arm_qy", "right_arm_qz", "right_arm_qw",
                            "head_pan", "head_tilt", "torso_lift", "base_linear_x", "base_linear_y", "base_angular_z"])
            
        with open(self.base_trajectory_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw"])

        with open(self.base_to_target_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "target1_x", "target1_y", "target1_z", "target1_qx", "target1_qy", "target1_qz", "target1_qw",
                             "target2_x", "target2_y", "target2_z", "target2_qx", "target2_qy", "target2_qz", "target2_qw",
                             "target3_x", "target3_y", "target3_z", "target3_qx", "target3_qy", "target3_qz", "target3_qw",
                             "target4_x", "target4_y", "target4_z", "target4_qx", "target4_qy", "target4_qz", "target4_qw",
                             "target5_x", "target5_y", "target5_z", "target5_qx", "target5_qy", "target5_qz", "target5_qw",
                             "target6_x", "target6_y", "target6_z", "target6_qx", "target6_qy", "target6_qz", "target6_qw",
                             "target7_x", "target7_y", "target7_z", "target7_qx", "target7_qy", "target7_qz", "target7_qw",
                             "target8_x", "target8_y", "target8_z", "target8_qx", "target8_qy", "target8_qz", "target8_qw",
                             "target9_x", "target9_y", "target9_z", "target9_qx", "target9_qy", "target9_qz", "target9_qw",
                             "target10_x", "target10_y", "target10_z", "target10_qx", "target10_qy", "target10_qz", "target10_qw"])
            
        with open(self.arm_left_to_target_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "target1_x", "target1_y", "target1_z", "target1_qx", "target1_qy", "target1_qz", "target1_qw",
                             "target2_x", "target2_y", "target2_z", "target2_qx", "target2_qy", "target2_qz", "target2_qw",
                             "target3_x", "target3_y", "target3_z", "target3_qx", "target3_qy", "target3_qz", "target3_qw",
                             "target4_x", "target4_y", "target4_z", "target4_qx", "target4_qy", "target4_qz", "target4_qw",
                             "target5_x", "target5_y", "target5_z", "target5_qx", "target5_qy", "target5_qz", "target5_qw",
                             "target6_x", "target6_y", "target6_z", "target6_qx", "target6_qy", "target6_qz", "target6_qw",
                             "target7_x", "target7_y", "target7_z", "target7_qx", "target7_qy", "target7_qz", "target7_qw",
                             "target8_x", "target8_y", "target8_z", "target8_qx", "target8_qy", "target8_qz", "target8_qw",
                             "target9_x", "target9_y", "target9_z", "target9_qx", "target9_qy", "target9_qz", "target9_qw",
                             "target10_x", "target10_y", "target10_z", "target10_qx", "target10_qy", "target10_qz", "target10_qw",
                             "target11_x", "target11_y", "target11_z", "target11_qx", "target11_qy", "target11_qz", "target11_qw",
                             "target12_x", "target12_y", "target12_z", "target12_qx", "target12_qy", "target12_qz", "target12_qw"])
            
        with open(self.arm_right_to_target_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "target1_x", "target1_y", "target1_z", "target1_qx", "target1_qy", "target1_qz", "target1_qw",
                             "target2_x", "target2_y", "target2_z", "target2_qx", "target2_qy", "target2_qz", "target2_qw",
                             "target3_x", "target3_y", "target3_z", "target3_qx", "target3_qy", "target3_qz", "target3_qw",
                             "target4_x", "target4_y", "target4_z", "target4_qx", "target4_qy", "target4_qz", "target4_qw",
                             "target5_x", "target5_y", "target5_z", "target5_qx", "target5_qy", "target5_qz", "target5_qw",
                             "target6_x", "target6_y", "target6_z", "target6_qx", "target6_qy", "target6_qz", "target6_qw",
                             "target7_x", "target7_y", "target7_z", "target7_qx", "target7_qy", "target7_qz", "target7_qw",
                             "target8_x", "target8_y", "target8_z", "target8_qx", "target8_qy", "target8_qz", "target8_qw",
                             "target9_x", "target9_y", "target9_z", "target9_qx", "target9_qy", "target9_qz", "target9_qw",
                             "target10_x", "target10_y", "target10_z", "target10_qx", "target10_qy", "target10_qz", "target10_qw",
                             "target11_x", "target11_y", "target11_z", "target11_qx", "target11_qy", "target11_qz", "target11_qw",
                             "target12_x", "target12_y", "target12_z", "target12_qx", "target12_qy", "target12_qz", "target12_qw"])
        
        with open(self.controller_states_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "right_hand_x", "right_hand_y", "right_hand_z", "right_hand_qx", "right_hand_qy", "right_hand_qz", "right_hand_qw",
                             "left_hand_x", "left_hand_y", "left_hand_z", "left_hand_qx", "left_hand_qy", "left_hand_qz", "left_hand_qw",
                             "head_x", "head_y", "head_z", "head_qx", "head_qy", "head_qz", "head_qw",
                             "right_axes_1", "right_axes_2", "right_axes_3", "right_buttons_1", "right_buttons_2", "right_buttons_3", "right_buttons_4",
                             "left_axes_1", "left_axes_2", "left_axes_3", "left_buttons_1", "left_buttons_2", "left_buttons_3", "left_buttons_4"])
            
        ## -------------- Variables -------------- ##
        self.listener = tf.TransformListener()
        self.start_time = rospy.Time.now().to_sec()
        self.origin = "odom"
        self.robot_base_link = "base_footprint"
        self.arm_base_link = "torso_lift_link"
        self.arm_ee_link = ["gripper_left_grasping_frame", "gripper_right_grasping_frame"]
        
        self.object_frames = ["yellow_can", "yellow_target", "orange_can", "orange_target", "hat", "hat_target", "clothes", "clothes_target",
                            "basket_right_handle", "basket_left_handle", "basket_right_target", "basket_left_target"]
        
        self.torso_joint = 0.0
        self.head_joint = [0.0, 0.0]
        self.base_velocity = [0.0, 0.0, 0.0]

        self.left_robot_activation = 0.0
        self.right_robot_activation = 0.0
        self.right_hand_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.left_hand_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.head_pose = {
            'position': np.array([0.0, 0.0, 0.0]),
            'orientation': np.array([1.0, 0.0, 0.0, 0.0]),
        }
        self.right_buttons = {
            'axes': np.array([0.0, 0.0, 0.0]),
            'buttons': np.array([0.0, 0.0, 0.0, 0.0]),
        }
        self.left_buttons = {
            'axes': np.array([0.0, 0.0, 0.0]),
            'buttons': np.array([0.0, 0.0, 0.0, 0.0]),
        }
        self.left_extend = False
        self.right_extend = False

        rospy.sleep(0.5)
        
        ## -------------- Subscribers -------------- ##
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self.__base_callback)
        rospy.Subscriber('/left/robot_activation', Float64, self.__left_robot_activation_callback)
        rospy.Subscriber('/right/robot_activation', Float64, self.__right_robot_activation_callback)
        rospy.Subscriber('/Right_Hand', TransformStamped, self.__right_input_pose_callback)
        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        rospy.Subscriber('/Left_Hand', TransformStamped, self.__left_input_pose_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Head_Motion', PoseStamped, self.__head_motion_callback)
        rospy.Subscriber('/left/arm_extend', Bool, self.__left_arm_extend_callback)
        rospy.Subscriber('/right/arm_extend', Bool, self.__right_arm_extend_callback)

    ## -------------- Callbacks -------------- ##

    def __joint_states_callback(self, msg):

        self.torso_joint = msg.position[20]
        self.head_joint = [msg.position[18], msg.position[19]]

    def __base_callback(self, msg):
        self.base_velocity = [msg.linear.x, msg.linear.y, msg.angular.z]

    def __left_robot_activation_callback(self, msg):
        self.left_robot_activation = msg.data

    def __right_robot_activation_callback(self, msg):
        self.right_robot_activation = msg.data

    def __right_input_pose_callback(self, msg):
        self.right_hand_pose['position'] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        self.right_hand_pose['orientation'] = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

    def __right_input_buttons_callback(self, msg):
        self.right_buttons['axes'] = np.array(msg.axes)
        self.right_buttons['buttons'] = np.array(msg.buttons)

    def __left_input_pose_callback(self, msg):
        self.left_hand_pose['position'] = np.array([msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z])
        self.left_hand_pose['orientation'] = np.array([msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w])

    def __left_input_buttons_callback(self, msg):
        self.left_buttons['axes'] = np.array(msg.axes)
        self.left_buttons['buttons'] = np.array(msg.buttons)

    def __head_motion_callback(self, msg):
        self.head_pose['position'] = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.head_pose['orientation'] = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    def __left_arm_extend_callback(self, msg):
        self.left_extend = msg.data

    def __right_arm_extend_callback(self, msg):
        self.right_extend = msg.data

    ## -------------- Helpers -------------- ##
    def __make_sure_path_exists(self, path):
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)


    def __get_arm_transformation(self, side):
        try:
            ee_link = self.arm_ee_link[0] if side == "left" else self.arm_ee_link[1]
            (trans, rot) = self.listener.lookupTransform(self.arm_base_link, ee_link, rospy.Time(0))
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
            return None

    def __get_transformation(self, parent, child):
        try:
            (trans, rot) = self.listener.lookupTransform(parent, child, rospy.Time(0))
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
            return None

    ## -------------- save to csv -------------- ##

    def robot_states_save_to_csv(self):
        filename = self.robot_states_filename

        left_arm_current_pose = self.__get_arm_transformation("left")
        right_arm_current_pose = self.__get_arm_transformation("right")
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, self.left_robot_activation, self.left_extend, left_arm_current_pose.position.x, left_arm_current_pose.position.y, left_arm_current_pose.position.z,
                            left_arm_current_pose.orientation.x, left_arm_current_pose.orientation.y, left_arm_current_pose.orientation.z, left_arm_current_pose.orientation.w,
                            self.right_robot_activation, self.right_extend, right_arm_current_pose.position.x, right_arm_current_pose.position.y, right_arm_current_pose.position.z,
                            right_arm_current_pose.orientation.x, right_arm_current_pose.orientation.y, right_arm_current_pose.orientation.z, right_arm_current_pose.orientation.w,
                            self.head_joint[0], self.head_joint[1], self.torso_joint, self.base_velocity[0], self.base_velocity[1], self.base_velocity[2]])
    
    def arm_to_target_save_to_csv(self, side):
        filename = self.arm_left_to_target_filename if side == "left" else self.arm_right_to_target_filename
        arm_link = self.arm_ee_link[0] if side == "left" else self.arm_ee_link[1]
        arm_to_target = []
        for i in range(len(self.object_frames)):
            arm_to_target.append(self.__get_transformation(arm_link, self.object_frames[i]))

        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, arm_to_target[0].position.x, arm_to_target[0].position.y, arm_to_target[0].position.z,
                             arm_to_target[0].orientation.x, arm_to_target[0].orientation.y, arm_to_target[0].orientation.z, arm_to_target[0].orientation.w,
                             arm_to_target[1].position.x, arm_to_target[1].position.y, arm_to_target[1].position.z,
                             arm_to_target[1].orientation.x, arm_to_target[1].orientation.y, arm_to_target[1].orientation.z, arm_to_target[1].orientation.w,
                             arm_to_target[2].position.x, arm_to_target[2].position.y, arm_to_target[2].position.z,
                             arm_to_target[2].orientation.x, arm_to_target[2].orientation.y, arm_to_target[2].orientation.z, arm_to_target[2].orientation.w,
                             arm_to_target[3].position.x, arm_to_target[3].position.y, arm_to_target[3].position.z,
                             arm_to_target[3].orientation.x, arm_to_target[3].orientation.y, arm_to_target[3].orientation.z, arm_to_target[3].orientation.w,
                             arm_to_target[4].position.x, arm_to_target[4].position.y, arm_to_target[4].position.z,
                             arm_to_target[4].orientation.x, arm_to_target[4].orientation.y, arm_to_target[4].orientation.z, arm_to_target[4].orientation.w,
                             arm_to_target[5].position.x, arm_to_target[5].position.y, arm_to_target[5].position.z,
                             arm_to_target[5].orientation.x, arm_to_target[5].orientation.y, arm_to_target[5].orientation.z, arm_to_target[5].orientation.w,
                             arm_to_target[6].position.x, arm_to_target[6].position.y, arm_to_target[6].position.z,
                             arm_to_target[6].orientation.x, arm_to_target[6].orientation.y, arm_to_target[6].orientation.z, arm_to_target[6].orientation.w,
                             arm_to_target[7].position.x, arm_to_target[7].position.y, arm_to_target[7].position.z,
                             arm_to_target[7].orientation.x, arm_to_target[7].orientation.y, arm_to_target[7].orientation.z, arm_to_target[7].orientation.w,
                             arm_to_target[8].position.x, arm_to_target[8].position.y, arm_to_target[8].position.z,
                             arm_to_target[8].orientation.x, arm_to_target[8].orientation.y, arm_to_target[8].orientation.z, arm_to_target[8].orientation.w,
                             arm_to_target[9].position.x, arm_to_target[9].position.y, arm_to_target[9].position.z,
                             arm_to_target[9].orientation.x, arm_to_target[9].orientation.y, arm_to_target[9].orientation.z, arm_to_target[9].orientation.w,
                             arm_to_target[10].position.x, arm_to_target[10].position.y, arm_to_target[10].position.z,
                             arm_to_target[10].orientation.x, arm_to_target[10].orientation.y, arm_to_target[10].orientation.z, arm_to_target[10].orientation.w,
                             arm_to_target[11].position.x, arm_to_target[11].position.y, arm_to_target[11].position.z,
                             arm_to_target[11].orientation.x, arm_to_target[11].orientation.y, arm_to_target[11].orientation.z, arm_to_target[11].orientation.w
                             ])

    def robot_base_trajectory_save_to_csv(self):
        filename = self.base_trajectory_filename
        base_current_pose = self.__get_transformation(self.origin, self.robot_base_link)
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, base_current_pose.position.x, base_current_pose.position.y, base_current_pose.position.z,
                            base_current_pose.orientation.x, base_current_pose.orientation.y, base_current_pose.orientation.z, base_current_pose.orientation.w])

    def base_to_target_save_to_csv(self):
        filename = self.base_to_target_filename
        base_to_target = []
        for i in range(len(self.object_frames)):
            base_to_target.append(self.__get_transformation(self.robot_base_link, self.object_frames[i]))

        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, base_to_target[0].position.x, base_to_target[0].position.y, base_to_target[0].position.z,
                             base_to_target[0].orientation.x, base_to_target[0].orientation.y, base_to_target[0].orientation.z, base_to_target[0].orientation.w,
                             base_to_target[1].position.x, base_to_target[1].position.y, base_to_target[1].position.z,
                             base_to_target[1].orientation.x, base_to_target[1].orientation.y, base_to_target[1].orientation.z, base_to_target[1].orientation.w,
                             base_to_target[2].position.x, base_to_target[2].position.y, base_to_target[2].position.z,
                             base_to_target[2].orientation.x, base_to_target[2].orientation.y, base_to_target[2].orientation.z, base_to_target[2].orientation.w,
                             base_to_target[3].position.x, base_to_target[3].position.y, base_to_target[3].position.z,
                             base_to_target[3].orientation.x, base_to_target[3].orientation.y, base_to_target[3].orientation.z, base_to_target[3].orientation.w,
                             base_to_target[4].position.x, base_to_target[4].position.y, base_to_target[4].position.z,
                             base_to_target[4].orientation.x, base_to_target[4].orientation.y, base_to_target[4].orientation.z, base_to_target[4].orientation.w,
                             base_to_target[5].position.x, base_to_target[5].position.y, base_to_target[5].position.z,
                             base_to_target[5].orientation.x, base_to_target[5].orientation.y, base_to_target[5].orientation.z, base_to_target[5].orientation.w,
                             base_to_target[6].position.x, base_to_target[6].position.y, base_to_target[6].position.z,
                             base_to_target[6].orientation.x, base_to_target[6].orientation.y, base_to_target[6].orientation.z, base_to_target[6].orientation.w,
                             base_to_target[7].position.x, base_to_target[7].position.y, base_to_target[7].position.z,
                             base_to_target[7].orientation.x, base_to_target[7].orientation.y, base_to_target[7].orientation.z, base_to_target[7].orientation.w,
                             base_to_target[8].position.x, base_to_target[8].position.y, base_to_target[8].position.z,
                             base_to_target[8].orientation.x, base_to_target[8].orientation.y, base_to_target[8].orientation.z, base_to_target[8].orientation.w,
                             base_to_target[9].position.x, base_to_target[9].position.y, base_to_target[9].position.z,
                             base_to_target[9].orientation.x, base_to_target[9].orientation.y, base_to_target[9].orientation.z, base_to_target[9].orientation.w,
                             base_to_target[10].position.x, base_to_target[10].position.y, base_to_target[10].position.z,
                             base_to_target[10].orientation.x, base_to_target[10].orientation.y, base_to_target[10].orientation.z, base_to_target[10].orientation.w,
                             base_to_target[11].position.x, base_to_target[11].position.y, base_to_target[11].position.z,
                             base_to_target[11].orientation.x, base_to_target[11].orientation.y, base_to_target[11].orientation.z, base_to_target[11].orientation.w])

    def controller_states_save_to_csv(self):
        filename = self.controller_states_filename
        t = rospy.Time.now().to_sec() - self.start_time
        with open(filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([t, self.right_hand_pose['position'][0], self.right_hand_pose['position'][1], self.right_hand_pose['position'][2],
                             self.right_hand_pose['orientation'][0], self.right_hand_pose['orientation'][1], self.right_hand_pose['orientation'][2], self.right_hand_pose['orientation'][3],
                             self.left_hand_pose['position'][0], self.left_hand_pose['position'][1], self.left_hand_pose['position'][2],
                             self.left_hand_pose['orientation'][0], self.left_hand_pose['orientation'][1], self.left_hand_pose['orientation'][2], self.left_hand_pose['orientation'][3],
                             self.head_pose['position'][0], self.head_pose['position'][1], self.head_pose['position'][2],
                             self.head_pose['orientation'][0], self.head_pose['orientation'][1], self.head_pose['orientation'][2], self.head_pose['orientation'][3],
                             self.right_buttons['axes'][0], self.right_buttons['axes'][1], self.right_buttons['axes'][2],
                             self.right_buttons['buttons'][0], self.right_buttons['buttons'][1], self.right_buttons['buttons'][2], self.right_buttons['buttons'][3],
                             self.left_buttons['axes'][0], self.left_buttons['axes'][1], self.left_buttons['axes'][2],
                             self.left_buttons['buttons'][0], self.left_buttons['buttons'][1], self.left_buttons['buttons'][2], self.left_buttons['buttons'][3]])

    ## -------------- Main Loop -------------- ##
    def run(self):
        self._running = True

        while self._running and not rospy.is_shutdown():

            self.robot_states_save_to_csv()
            self.robot_base_trajectory_save_to_csv()
            self.base_to_target_save_to_csv()
            self.arm_to_target_save_to_csv("left")
            self.arm_to_target_save_to_csv("right")
            self.controller_states_save_to_csv()
            rospy.sleep(0.05)

def node_shutdown():
    print("Shutting down data_recorder node")
    

## -------------- Main Function -------------- ## 
def main():
    try:
        rospy.init_node('data_recorder')
        rospy.on_shutdown(node_shutdown)

        app = DataRecorder()
        app.run()

    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()

