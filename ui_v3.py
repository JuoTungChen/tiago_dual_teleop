import threading
import cv2
import os
import numpy as np
import rospy
import tf2_ros
import tf.transformations
import math
import cv2.aruco as aruco
from sensor_msgs.msg import Joy, Image, JointState, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped, Pose
from std_msgs.msg import String, Float64, Bool, Int32
import argparse
import time
import datetime

# Parse command-line arguments
parser = argparse.ArgumentParser(description='Run the script with optional timer.')
parser.add_argument('--timer', action='store_true', help='Include to run with a timer')
args = parser.parse_args()

class videoThread(threading.Thread):
    def __init__(self, threadID, name, ip_addr):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.rgb_img = np.zeros((480, 640, 3), np.uint8) # (960, 540, 3)

        self.__tracking_button = 0
        self.__mode_button = 0

        self.controller_x = 0
        self.controller_y = 0
        self.controller_z = 0

        self.right_act = False
        self.left_act = False
        self.right_status_text = "Pause"
        self.right_status_color = (100, 100, 255)
        self.left_status_text = "Pause"
        self.left_status_color = (100, 100, 255)
        # Initialize alpha value
        self.alpha = 0.5
        self.prev_blended = None

        self.torso_height_bar_length = 50  # Length of the bar
        self.torso_height_bar_thickness = 15  # Thickness of the bar
        self.bar_color = (0, 255, 0)  # Color of the bar
        self.torso_height = 0.0
        self.min_torso_height = 0.05  # Minimum torso height
        self.max_torso_height = 0.35  # Maximum torso height

        # Initialize warning state
        self.warning_color = (0, 0, 255)  # Red color for warning
        self.warning_color_mid = (0, 255, 255)  # Yellow color for mid warning
        self.warning_thickness = 5  # Thickness of the warning lines
        self.warning_thickness_mid = 2  # Thickness of the warning lines
        self.warning_flash_interval = 20  # Flashing interval (in frames)
        # Number of sections
        self.num_sections = 8
        self.warnings = [False] * self.num_sections
        self.warnings_mid = [False] * self.num_sections

        # Section angles (in radians)
        self.section_angles = np.linspace(-np.pi, np.pi, self.num_sections + 1)
        # Counter for flashing effect
        self.flash_counter = 0

        # LiDAR data
        self.scan_data = None
        self.point_cloud = None
        self.point_cloud_lock = threading.Lock()

        # Initialize LiDAR parameters
        self.lidar_max_range = 5.0  # Maximum range of the LiDAR in meters
        self.collision_threshold = 0.55  # Threshold distance for collision detection in meters
        self.collision_threshold_mid = 1.0  # Threshold distance for collision detection in meters
        self.ranges = np.array([])

        self.distances = [np.array([]) for _ in range(self.num_sections)]
        self.lidar_warning_range = 1.5  # Range of the LiDAR warning in pixels
        self.lidar_warning_scale = 40
        self.left_arm_warnings = False
        self.right_arm_warnings = False
        self.left_text_pos = [130, 350]
        self.right_text_pos = [420, 350]
        self.started = False
        self.init_time = time.time()
        self.time_limit = 300
        self.time_warning = 120
        self.time_color = (51, 255, 255)
        self.time_up = False
        self.timer_pos = (210, 100)

        self.cap_rs2 = cv2.VideoCapture(
            'v4l2src device=/dev/video0 ! videoscale ! video/x-raw,width=640,height=480,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)
        # self.cap_rs3 = cv2.VideoCapture(
        #     'v4l2src device=/dev/video10 ! videoscale ! video/x-raw,width=1600,height=900,framerate=30/1 ! decodebin ! videoconvert ! appsink', cv2.CAP_GSTREAMER)

        
        self.out_send = cv2.VideoWriter(
            'appsrc! videoconvert ! video/x-raw,format=YUY2 ! jpegenc! rtpjpegpay ! udpsink host=192.168.0.173 port=2337 sync=false',
            cv2.CAP_GSTREAMER, 0, 30, (640, 480)) # (960, 540)
        
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Use lowercase 'mp4v' instead of 'MP4V'

        self.out_save = None
        
        self.running = True

        rospy.init_node('user_interface')
        # KINOVA eye-in-hand camera ########################
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.callback_image)
        # control state ########################
        rospy.Subscriber('/robot_activation', Float64, self.__tracking_button_callback, queue_size=1)
        rospy.Subscriber('/right/robot_activation', Float64, self.__right_activation_callback, queue_size=1)
        rospy.Subscriber('/left/robot_activation', Float64, self.__left_activation_callback, queue_size=1)
        # controller feedback
        rospy.Subscriber('/Right_Hand', TransformStamped, self.__input_pose_callback)
        rospy.Subscriber('/joint_states', JointState, self.__joint_states_callback)
        rospy.Subscriber("/scan", LaserScan, self.__scan_callback)
        rospy.Subscriber('/left/arm_extend', Bool, self.__left_arm_extend_callback)
        rospy.Subscriber('/right/arm_extend', Bool, self.__right_arm_extend_callback)
        rospy.Subscriber('/Right_Buttons', Joy, self.__right_input_buttons_callback)
        rospy.Subscriber('/Left_Buttons', Joy, self.__left_input_buttons_callback)
        rospy.Subscriber('/Task_Num', Int32, self.__task_num_callback)
        rospy.Subscriber('/Timestamp', String, self.__timestamp_callback)

    ## ------------------- Callback functions ------------------- ##
    def callback_image(self, data):
        # global rgb_img
        self.rgb_img = CvBridge().imgmsg_to_cv2(data, "bgr8")

    def __scan_callback(self, data):
        # Store the received scan data
        self.scan_data = data
        if self.scan_data is not None:
            # Convert scan data to numpy array
            self.ranges = np.array(self.scan_data.ranges)

    def __joint_states_callback(self, msg):

        self.torso_height = msg.position[20]

    def __left_arm_extend_callback(self, msg):
        if msg.data:
            self.left_arm_warnings = True
        else:
            self.left_arm_warnings = False

    def __right_arm_extend_callback(self, msg):
        if msg.data:
            self.right_arm_warnings = True
        else:
            self.right_arm_warnings = False

    def __tracking_button_callback(self, data):

        self.__tracking_button = data.data

    def __mode_button_callback(self, data):

        self.__mode_button = data.data

    def __input_pose_callback(self, msg):

        self.controller_x = msg.transform.translation.x
        self.controller_y = msg.transform.translation.y
        self.controller_z = msg.transform.translation.z

    def __right_activation_callback(self, msg):
        if msg.data == 2.0 and not self.right_act:
            self.right_act = True
            self.right_status_text = "Controlling"
            self.right_status_color = (0, 255, 0)

        if msg.data == 0.0 and self.right_act:
            self.right_act = False
            self.right_status_text = "Pause"
            self.right_status_color = (100, 100, 255)

    def __left_activation_callback(self, msg):
        if msg.data == 2.0 and not self.left_act:
            self.left_act = True
            self.left_status_text = "Controlling"
            self.left_status_color = (0, 255, 0)
        if msg.data == 0.0 and self.left_act:
            self.left_act = False
            self.left_status_text = "Pause"
            self.left_status_color = (100, 100, 255)

    def __right_input_buttons_callback(self, msg):
        for button_state in msg.buttons:
            if button_state == 1 and not self.started:  # 1 is typically the state for a button press
                self.started = True
                self.init_time = time.time()
                break  # No need to check the rest of the buttons
    
    def __left_input_buttons_callback(self, msg):
        for button_state in msg.buttons:
            if button_state == 1 and not self.started:  # 1 is typically the state for a button press
                self.started = True
                self.init_time = time.time()

                break  # No need to check the rest of the buttons

    def __task_num_callback(self, msg):
        rospy.logwarn(msg)

        self.task_num = msg.data

    def __timestamp_callback(self, msg):
        rospy.logwarn(msg)
        self.vid_name = os.path.expanduser(f"~/RAL2024/teleop_data/{msg.data}/task{self.task_num}/ui.mp4")
        self.__make_sure_path_exists(self.vid_name)
        self.out_save = cv2.VideoWriter(self.vid_name, self.fourcc, 30, (640,480))

    def __make_sure_path_exists(self, path):
        directory = os.path.dirname(path)
        if not os.path.exists(directory):
            os.makedirs(directory)

    def detect_markers(self, out, id):

        try:

            aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL) 
            arucoParameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(out, aruco_dict, parameters=arucoParameters)
            # print(ids)
            value = np.where(ids==id)
            corner = corners[value[0][0]]
            mid_x_ar = int((corner[0][0][0]+corner[0][1][0])/2)
            mid_y_ar = int((corner[0][0][1]+corner[0][2][1])/2)

        except:
            return 0, 0, 0

        return mid_x_ar, mid_y_ar, corner

    def dist_check(self, corners, threshold):
        
        return_val = 0
        try:

            if abs(corners[0][0][1] - corners[0][2][1]) > threshold:

                return_val = 1
            
            elif abs(corners[0][1][0] - corners[0][3][0]) > threshold:

                return_val = 1
        except:
            return 0
        return return_val

    def draw_warnings(self, frame):
        if self.time_up and self.flash_counter < self.warning_flash_interval:
            cv2.putText(frame, 'Time\'s Up', (150, 200), cv2.FONT_HERSHEY_SIMPLEX, 4, (20, 20, 230), 7, cv2.LINE_AA)

        ## Arm extension warnings
        if self.right_arm_warnings and self.flash_counter < self.warning_flash_interval:
            cv2.putText(frame, 'Limit', (self.right_text_pos[0]+20, self.right_text_pos[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        if self.left_arm_warnings and self.flash_counter < self.warning_flash_interval:
            cv2.putText(frame, 'Limit', (self.left_text_pos[0]+20, self.left_text_pos[1]-20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2, cv2.LINE_AA)

        ## LiDAR warnings
        for i in range(0, len(self.ranges), 3):
            distance = self.ranges[i]
            if distance < self.lidar_warning_range:
                # print(self.scan_data.angle_min)
                angle =  self.scan_data.angle_increment * i
                x = int(distance * 40 * np.sin(angle)) - 8
                y = int(distance * 50 * np.cos(angle))
                # print(x, y)
                if distance < self.collision_threshold:
                    self.warning_color = (0, 0, 255)
                else:
                    self.warning_color = (0, 255, 0)
                # Draw the dot
                cv2.circle(frame, (360+x, 375+y), 2, self.warning_color, 2)
                # if i == 0:
                #     # print(self.distances[i])
                #     cv2.line(frame, (352, 385 + int(self.distances[i] * self.lidar_warning_scale)), (368, 385 + int(self.distances[i] * self.lidar_warning_scale)), self.warning_color, self.warning_thickness)
                # elif i == 1:
                #     cv2.line(frame, (360 + int(self.distances[i] * self.lidar_warning_scale), 398), (360 + int(self.distances[i] * self.lidar_warning_scale), 377), self.warning_color, self.warning_thickness)
                # elif i == 2:
                #     cv2.line(frame, (360 + int(self.distances[i] * self.lidar_warning_scale), 373), (360 + int(self.distances[i] * self.lidar_warning_scale), 352), self.warning_color, self.warning_thickness)
                # elif i == 3:
                #     cv2.line(frame, (352, 360 - int(self.distances[i] * self.lidar_warning_scale)), (368, 360 - int(self.distances[i] * self.lidar_warning_scale)), self.warning_color, self.warning_thickness)
                # elif i == 4:
                #     cv2.line(frame, (332, 360 - int(self.distances[i] * self.lidar_warning_scale)), (348, 360 - int(self.distances[i] * self.lidar_warning_scale)), self.warning_color, self.warning_thickness)
                # elif i == 5:
                #     cv2.line(frame, (345 - int(self.distances[i] * self.lidar_warning_scale), 352), (345 - int(self.distances[i] * self.lidar_warning_scale), 373), self.warning_color, self.warning_thickness)
                # elif i == 6:
                #     cv2.line(frame, (345 - int(self.distances[i] * self.lidar_warning_scale), 377), (345 - int(self.distances[i] * self.lidar_warning_scale), 398), self.warning_color, self.warning_thickness)
                # elif i == 7:
                #     cv2.line(frame, (332, 385 + int(self.distances[i] * self.lidar_warning_scale)), (348, 385 + int(self.distances[i] * self.lidar_warning_scale)), self.warning_color, self.warning_thickness)

            # elif self.warnings_mid[i]:
            #     if i == 0:
            #         cv2.line(frame, (352, 405), (368, 405), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 1:
            #         cv2.line(frame, (375, 398), (375, 377), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 2:
            #         cv2.line(frame, (375, 373), (375, 352), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 3:
            #         cv2.line(frame, (352, 345), (368, 345), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 4:
            #         cv2.line(frame, (332, 345), (348, 345), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 5:
            #         cv2.line(frame, (325, 352), (325, 373), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 6:
            #         cv2.line(frame, (325, 377), (325, 398), self.warning_color_mid, self.warning_thickness_mid)
            #     elif i == 7:
            #         cv2.line(frame, (332, 405), (348, 405), self.warning_color_mid, self.warning_thickness_mid)
        
        # Increment flash counter and reset if necessary
        # self.flash_counter += 1
        # if self.flash_counter >= 2 * self.warning_flash_interval:
        #     self.flash_counter = 0


    def format_time(self, seconds):
        """
            Function to format seconds into HH:MM:SS
        """
        m, s = divmod(seconds, 60)
        # h, m = divmod(m, 60)
        return "{:02}:{:02}".format(int(m), int(s))
    
    def run(self):
        
        while not rospy.is_shutdown():
            
            
            # specify eye-in-hand camera
            frame_rs1 = self.rgb_img

            ##------------- display control status -------------##
            cv2.putText(frame_rs1, self.right_status_text, (self.right_text_pos[0], self.right_text_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.right_status_color, 2, cv2.LINE_AA)
            cv2.putText(frame_rs1, self.left_status_text, (self.left_text_pos[0], self.left_text_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.left_status_color, 2, cv2.LINE_AA)
            
            ##------------- draw torso status -------------##
            bar_position_y = 350  # Vertical position of the bar
            bar_position_x = 260  # Vertical position of the bar
            cv2.rectangle(frame_rs1, (bar_position_x, bar_position_y), (bar_position_x + self.torso_height_bar_thickness, bar_position_y + self.torso_height_bar_length), self.bar_color, 2)
            line_position_y = int(bar_position_y+self.torso_height_bar_length - ((self.torso_height - self.min_torso_height) / (self.max_torso_height - self.min_torso_height)) * self.torso_height_bar_length)
            cv2.line(frame_rs1, (bar_position_x, line_position_y), (bar_position_x + self.torso_height_bar_thickness, line_position_y), (0, 255, 0), 5)
            

            ##------------- draw lidar warnings -------------##
            cv2.rectangle(frame_rs1, (340, 360), (360, 390), (255,255,255), 5)
            # cv2.putText(frame_rs1, 'Base', (332, 377), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (127,127,127), 1, cv2.LINE_AA)
            self.draw_warnings(frame_rs1)


            ##------------- display time -------------##
            if args.timer:
                if self.started:
                    elapsed = time.time() - self.init_time
                    # Calculate the remaining time
                    remaining = self.time_limit - elapsed
                    if remaining <= 0:
                        remaining = 0
                        self.time_up = True
                    else:
                        if remaining < self.time_warning:
                            self.time_color = (20, 20, 230)
                        formatted_time = self.format_time(remaining)
                        time_text = f"Time: {formatted_time}"
                        cv2.putText(frame_rs1, time_text, self.timer_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, self.time_color, 2, cv2.LINE_AA)
            else:
                pass
                # if self.started:
                #     elapsed = time.time() - self.init_time
                #     formatted_time = self.format_time(elapsed)
                #     time_text = f"Time: {formatted_time}"
                #     cv2.putText(frame_rs1, time_text, self.timer_pos, cv2.FONT_HERSHEY_SIMPLEX, 1, self.time_color, 2, cv2.LINE_AA)

            # display user interface       
            # frame_rs2_small = cv2.resize(frame_rs2, (200, 112), interpolation = cv2.INTER_AREA)

            # Replace the region of frame_rs1 with the blended image
            #frame_rs1[50:162, 150:350] = frame_rs2_small

            # Send out the visual feedback to unity
            self.out_send.write(frame_rs1)
            if self.out_save is not None:
                self.out_save.write(frame_rs1)
    
            cv2.namedWindow('Main View', cv2.WINDOW_GUI_NORMAL)
            cv2.resizeWindow('Main View', 640, 480)
            cv2.imshow('Main View', frame_rs1)
            time.sleep(0.03)

            # cv2.namedWindow('Eye-in-hand View', cv2.WINDOW_GUI_NORMAL)
            # cv2.resizeWindow('Eye-in-hand View', 960, 540)
            # cv2.imshow('Eye-in-hand View', frame_rs2)

            # subscibe keyborad input
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q'):
                break
            
        self.running = False
        # self.cap_rs1.release()
        # self.cap_rs2.release()
        self.out_send.release()
        if self.out_save is not None:

            self.out_save.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    ip_addr = '130.215.181.94'

    v_thread = videoThread(0, 'videoT', ip_addr)
    v_thread.start()
