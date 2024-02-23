#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
import actionlib
from teleop_tools_msgs.msg import IncrementAction as TTIA
from teleop_tools_msgs.msg import IncrementGoal as TTIG
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchController
from moveit_msgs.msg import MoveItErrorCodes
# from moveit_python import MoveGroupInterface, PlanningSceneInterface
import moveit_commander
class JoystickTeleop():
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.left_group_name = "arm_left_torso"
        self.left_move_group = moveit_commander.MoveGroupCommander(self.left_group_name)
        # self.left_move_group.setPlannerId("SBLkConfigDefault")
        self._pub_cmd = rospy.Publisher('key_vel', Twist, queue_size=10)
        self._joy_subscriber = rospy.Subscriber('joy', Joy, self._joy_callback)
        self.torso_client = actionlib.SimpleActionClient('/torso_controller/increment', TTIA)
        self.head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.left_gripper_client = actionlib.SimpleActionClient('/parallel_gripper_left_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.right_gripper_client = actionlib.SimpleActionClient('/parallel_gripper_right_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.left_arm_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=10)
        self.left_arm_client = actionlib.SimpleActionClient('/arm_left_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.gripper_pressed = False
        self._hz = rospy.get_param('~hz', 10)
        self._forward_rate = rospy.get_param('~forward_rate', 0.8)
        self._backward_rate = rospy.get_param('~backward_rate', 0.5)
        self._rotation_rate = rospy.get_param('~rotation_rate', 1.0)
        self.arm_side = "left"
        self._angular = 0
        self._linear = 0
        self._linear_y = 0
        self._mode = 'base'  # Initial mode is base control
        self.translation_mode = True
        self.vel_scale = 0.5
        self.rot_scale = 0.1
        self.start_controller=['arm_left_JGP']
        self.stop_controller=['arm_left_controller', 'torso_controller']
        self.arm_left_home_joint_angles = [0.1404634725889998, -1.0999085872589465, 1.4541173782490207, 2.720822327116011, 1.6583055087691845, -1.582990280917917, 1.3793348919241852, 0.0003437668173003061]
        rospy.sleep(0.5)

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running and not rospy.is_shutdown():
            rate.sleep()

    def _joy_callback(self, joy_msg):
        # Check button 2 for base mode and button 3 for head mode
        if joy_msg.buttons[2] and not self._mode == 'base':
            rospy.logwarn("changed to base mode")
            self._change_controller(self.stop_controller, self.start_controller)

            self._mode = 'base'
        elif joy_msg.buttons[3] and not self._mode == 'head':
            rospy.logwarn("changed to head mode")
            self._mode = 'head'
        elif joy_msg.buttons[1] and not self._mode == 'arm':
            rospy.logwarn("changed to arm mode")
            self._mode = 'arm'

            

        if (joy_msg.buttons[0] and self._mode == 'arm'):
            self.translation_mode = not self.translation_mode
            print("Translation mode:", self.translation_mode)
            rospy.sleep(0.1)

        # Check the current mode and execute the corresponding control
        if self._mode == 'base':
            
            self._base_control(joy_msg)
        elif self._mode == 'head':
            self._head_control(joy_msg)
        elif self._mode == 'arm':
            if(joy_msg.buttons[4] and not self.arm_side == "left"):
                self.arm_side = "left"
                self._change_controller(self.start_controller, self.stop_controller)
                rospy.logwarn("switched to left arm")
            elif(joy_msg.buttons[5] and not self.arm_side == "right"):
                self.arm_side = "right"
                rospy.logwarn("switched to right arm")
            if joy_msg.axes[6] == 1.0:
                rospy.logwarn("left arm move to home...")
                self._move_to_home_position()  
            elif joy_msg.axes[6] == -1.0:
                pass

            self._left_arm_teleop_control(joy_msg)
            # self._arm_cartesian_control(joy_msg)
            self._gripper_control(joy_msg)
            
        self._publish()

    def _base_control(self, joy_msg):
        # if joy_msg.button[0] and joy_msg.button[2]:

        # else:   
        linear = joy_msg.axes[1]
        linear_y = joy_msg.axes[0]
        angular = joy_msg.axes[3]
        self._linear = linear * self._forward_rate if linear > 0 else linear * self._backward_rate
        self._linear_y = linear_y * self._forward_rate if linear > 0 else linear_y * self._backward_rate
        self._angular = angular * self._rotation_rate

        # Check button 4 for torso up and button 5 for torso down
        if joy_msg.axes[7] == 1.0:
            self._torso_increment(0.05)  # Increment for torso up
        elif joy_msg.axes[7] == -1.0:
            self._torso_increment(-0.05)  # Increment for torso down



    def _head_control(self, joy_msg):
        # Adjust head position based on joystick axes
        if len(joy_msg.axes) >= 2:
            head_position = [joy_msg.axes[0], joy_msg.axes[1]]
            self._send_head_goal(head_position)

    def _gripper_control(self, joy_msg):
        if(not self.gripper_pressed and joy_msg.axes[5] != 0):
            self.gripper_pressed = True
        # Map joystick axis 5 to gripper position
        gripper_position = joy_msg.axes[5] * 0.04 + 0.04  

        # Create a trajectory point for the gripper
        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = [gripper_position]
        gripper_point.velocities = []
        gripper_point.accelerations = []
        gripper_point.effort = []
        gripper_point.time_from_start = rospy.Duration(1)  # Adjust the duration as needed

        # Create and send the gripper goal
        gripper_goal = FollowJointTrajectoryGoal()
        gripper_goal.trajectory.points = [gripper_point]
        gripper_goal.trajectory.header.stamp = rospy.Time.now()
        if(self.arm_side == "left"):
            gripper_goal.trajectory.joint_names = ["gripper_left_left_finger_joint", "gripper_left_right_finger_joint"]
            if self.gripper_pressed:
                self.left_gripper_client.send_goal(gripper_goal)
        elif(self.arm_side == "right"):
            gripper_goal.trajectory.joint_names = ["gripper_right_left_finger_joint", "gripper_right_right_finger_joint"]
            if self.gripper_pressed:
                self.right_gripper_client.send_goal(gripper_goal)

    def _arm_cartesian_control(self, joy_msg):
        # Map joystick axes to Cartesian coordinates
        x = joy_msg.axes[1]  # Axis 1 controls x
        y = joy_msg.axes[0]  # Axis 0 controls y
        z = joy_msg.axes[4]  # Axis 4 controls z

        # Get the current end-effector pose
        current_pose = self.left_move_group.get_current_pose().pose

        # Update the target pose based on joystick input
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.pose.position.x = current_pose.position.x + x
        target_pose.pose.position.y = current_pose.position.y + y
        target_pose.pose.position.z = current_pose.position.z + z
        target_pose.pose.orientation = current_pose.orientation
        self.left_move_group.set_pose_target(target_pose)
        success = self.left_move_group.go(wait=False)
        # Calling `stop()` ensures that there is no residual movement
        self.left_move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.left_move_group.clear_pose_targets()

    # def _left_arm_teleop_control(self, joy_msg):
    #     # Set the target pose for the left arm
    #     target_pose = self._calculate_target_pose(joy_msg)

    #     # Plan the trajectory
    #     self.left_move_group.set_pose_target(target_pose)
    #     plan = self.left_move_group.plan()

    #     # Check if the plan is valid
    #     if plan:
    #         # Publish the trajectory to the arm_controller/command topic
    #         # self._publish_trajectory(plan)
    #         self.left_move_group.go(wait=True)

    #     else:
    #         rospy.logwarn("Failed to plan trajectory")

    # def _calculate_target_pose(self, joy_msg):

    #     target_pose = PoseStamped()
    #     target_pose.header.frame_id = "base_footprint"  # Set the appropriate reference frame
    #     if(self.translation_mode):
    #         target_pose.pose.position.x = self.vel_scale* joy_msg.axes[1]  # Set the desired position based on joystick input
    #         target_pose.pose.position.y = self.vel_scale* joy_msg.axes[0]
    #         target_pose.pose.position.z = self.vel_scale* joy_msg.axes[4]
    #         target_pose.pose.orientation.w = 1.0  # Set the desired orientation
    #     else:
    #         target_pose.pose.position.x = 0.0
    #         target_pose.pose.position.y = 0.0
    #         target_pose.pose.position.z = 0.0
    #         target_pose.pose.orientation.x = self.rot_scale* joy_msg.axes[0]
    #         target_pose.pose.orientation.y = self.rot_scale* joy_msg.axes[1]
    #         target_pose.pose.orientation.z = self.rot_scale* joy_msg.axes[4]
    #     return target_pose
    
    # def _publish_trajectory(self, plan):
    #     # Extract the trajectory from the plan
    #     rospy.logwarn(plan)
    #     trajectory = plan
    #     self.left_arm_client.send_goal(trajectory)

    def _left_arm_teleop_control(self, joy_msg):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = rospy.Time.now()
        if(self.translation_mode):
            twist_msg.twist.linear.x = self.vel_scale* joy_msg.axes[1]  # Left stick up and down for controlling the x-axis
            twist_msg.twist.linear.y = self.vel_scale* joy_msg.axes[0]  # Left stick left and right for controlling the y-axis
            twist_msg.twist.linear.z = self.vel_scale* joy_msg.axes[4]  # Right stick up and down for controlling the z-axis
            twist_msg.twist.angular.x = 0.0
            twist_msg.twist.angular.y = 0.0
            twist_msg.twist.angular.z = 0.0
        else:
            twist_msg.twist.linear.x = 0.0
            twist_msg.twist.linear.y = 0.0
            twist_msg.twist.linear.z = 0.0
            twist_msg.twist.angular.x = self.rot_scale* joy_msg.axes[0]
            twist_msg.twist.angular.y = self.rot_scale* joy_msg.axes[1]
            twist_msg.twist.angular.z = self.rot_scale* joy_msg.axes[4]
        # end_effector_command = "speedl([{},{},{}], 0.1, 0.1)".format(twist.linear.x, twist.linear.y, twist.linear.z)  # Adjust the speed values as needed
        self.left_arm_pub.publish(twist_msg)

    def _change_controller(self, start_controller, stop_controller):
        timeout_value = 0.5
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            rospy.set_param('/controller_manager/incoming_command_timeout', timeout_value)
            switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            response = switch_controller(
                start_controllers=start_controller,
                stop_controllers=stop_controller,
                strictness=1,
            )
            rospy.logwarn("changed controller")
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed:", e)

    def _move_to_home_position(self):
        rospy.logwarn("moving to home position...")
        self._change_controller(self.stop_controller, self.start_controller)
        print("changed controller to eff")
        rospy.sleep(1)
        self.left_move_group.set_max_velocity_scaling_factor(0.5)
        self.left_move_group.set_max_acceleration_scaling_factor(0.5)
        # self.open_gripper()
        joint_goal= self.arm_left_home_joint_angles
        # curr_joint = self.left_move_group.get_current_joint_values()
        # rospy.logwarn(curr_joint)
        self.left_move_group.go(joint_goal,wait=True)
        self.left_move_group.stop()
        rospy.logwarn("moved to home")
        self._change_controller(self.start_controller, self.stop_controller)
        rospy.sleep(1)
        rospy.logwarn("changed controller to pos")


    def _send_head_goal(self, head_position):
        # Create a trajectory point with the adjusted head position
        point = JointTrajectoryPoint()
        point.positions = head_position
        point.velocities = [0.1, 0.1]  # Set desired joint velocities
        point.time_from_start = rospy.Duration(0.1)  # A small duration for smooth motion

        # Create and send the head goal
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory.joint_names = ["head_1_joint", "head_2_joint"]
        head_goal.trajectory.points = [point]
        head_goal.trajectory.header.stamp = rospy.Time.now()
        self.head_client.send_goal(head_goal)

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
        if self._mode == 'base':
            twist = self._get_twist(self._linear, self._linear_y, self._angular)
        else:
            twist = self._get_twist(0.0, 0.0, 0.0)
        self._pub_cmd.publish(twist)

if __name__ == '__main__':
    try:
        rospy.init_node('joystick_teleop')
        app = JoystickTeleop()
        app.run()
    except rospy.ROSInterruptException:
        pass
