#!/usr/bin/env python3

"""
Controller for the drone
* Team Id : 2686
* Author List : Srishivanth R F, Krishna Prasath S, Shyam M, Joseph Raj B
* Filename: species_seeker.py
* Theme: Luminosity Drone
* Functions: DroneController,whycon_poses_callback,pid_tune_throttle_callback,pid_tune_roll_callback,
            pid_tune_pitch_callback,at_setpoint,nextt,image_callback,aligning,next,pid,shutdown_hook,
            arm,disarm,now,main
* Global Variables: self.timer,self.lock,self.rc_message.aux1,self.rc_message.aux2,self.rc_message.aux3,
        self.rc_message.aux4,self.hover_duration,self.in_hover_mode,self.alien_type,
        self.biolocation_msg.organism_type,self.biolocation_msg.whycon_x,self.biolocation_msg.whycon_y,
        self.biolocation_msg.whycon_z,self.maxx_len,self.cx,self.cy,self.fx,self.fy,self.camera_x,
        self.camera_y,self.len_cnts,self.max_len,self.clear_block,self.list_alien,self.align_block,
        self.past_waypoint_idx,self.landd,self.buzzz_lock,self.buzzzz_lock,self.buz_lock,self.buzzzzz_lock,
        self.way_lock,self.waypoint,self.current_waypoint_indx,self.Base_z,self.Base_x,self.Base_y,
        self.earliest_z,self.hover_start,self.node,self.rc_message,self.rc_msg,self.drone_whycon_pose_array,
        self.last_whycon_pose_received_at,self.whycon_data_recieved,self.commandbool,service_endpoint,
        self.bridge,self.centroid_list,self.land_time,self.current_tim,self.cu_time,self.rc_throttle,
        self.rc_roll,self.rc_pitch,self.future,self.arming_service_client,self.error,self.integral,
        self.proportional,self.derrivative,self.prev_error,self.setpoint,self.Kp,self.Kd,self.Ki,
        self.drone_position,MIN_ROLL,MAX_ROLL,MIN_PITCH,MAX_PITCH,MIN_THROTTLE,MAX_THROTTLE,SUM_ERROR_ROLL_LIMIT,
        SUM_ERROR_PITCH_LIMIT,SUM_ERROR_THROTTLE_LIMIT,MAX_ROLL_FILTER,MIN_ROLL_FILTER,MAX_PITCH_FILTER,
        MIN_PITCH_FILTER,MAX_THROTTLE_FILTER,MIN_THROTTLE_FILTER,DRONE_WHYCON_POSE
"""

# standard imports
import copy
import time

# third-party imports
import scipy.signal
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PidTune
from swift_msgs.msg import PIDError, RCMessage
from swift_msgs.srv import CommandBool
from cv_bridge import CvBridge
from threading import Timer
from loc_msg.msg import Biolocation
from skimage import measure
from sensor_msgs.msg import Image
import cv2
import imutils
from rclpy.qos import QoSProfile



MIN_ROLL = 1400
MAX_ROLL = 1600

MIN_PITCH = 1400
MAX_PITCH = 1600

MIN_THROTTLE = 1400
MAX_THROTTLE = 1600

SUM_ERROR_ROLL_LIMIT = 4
SUM_ERROR_PITCH_LIMIT = 250
SUM_ERROR_THROTTLE_LIMIT = 10000

MAX_ROLL_FILTER = 2000
MIN_ROLL_FILTER = 1000

MAX_PITCH_FILTER = 2000
MIN_PITCH_FILTER = 1000

MAX_THROTTLE_FILTER = 2000
MIN_THROTTLE_FILTER = 1000

DRONE_WHYCON_POSE = [[], [], []]

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch
''' 
        * Function Name: whycon_poses_callback
        * Input: None
        * Output: self.timer,self.lock,self.rc_message.aux1,self.rc_message.aux2,self.rc_message.aux3,
                  self.rc_message.aux4,self.hover_duration,self.in_hover_mode,self.alien_type,
                  self.biolocation_msg.organism_type,self.biolocation_msg.whycon_x,self.biolocation_msg.whycon_y,
                  self.biolocation_msg.whycon_z,self.maxx_len,self.cx,self.cy,self.fx,self.fy,self.camera_x,
                  self.camera_y,self.len_cnts,self.max_len,self.clear_block
        * Logic: This block is the constructor method for the DroneController class. It initializes various attributes and sets up ROS subscriptions, publishers, and service clients.
        * Example Call: controller = DroneController(node)
'''
class DroneController():
    def __init__(self,node):
        self.node= node
        self.rc_message = RCMessage()
        self.rc_msg = RCMessage()
        self.drone_whycon_pose_array = PoseArray()
        self.last_whycon_pose_received_at = 0
        self.whycon_data_recieved = False
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        self.bridge=CvBridge()
        self.centroid_list=[]

        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)

        self.error = [0.0, 0.0, 0.0]         # Error for roll, pitch and throttle        
        self.integral = [0.0,0.0,0.0]
        self.proportional = [0.0,0.0,0.0]
        self.derrivative = [0.0,0.0,0.0]
        self.prev_error = [0.0,0.0,0.0]
        self.timer=True
        self.lock=True
        self.sum_error=[0,0,0]
        self.drone_position =[0,0,0]
        self.rc_message.aux1 = 1500
        self.rc_message.aux2 = 1500
        self.rc_message.aux3 = 1000
        self.rc_message.aux4 = 1000
        self.hover_duration = 0.7
        self.in_hover_mode = False

        self.alien_type=''
        self.biolocation_msg=Biolocation()
        self.biolocation_msg.organism_type=''
        self.biolocation_msg.whycon_x=0.0
        self.biolocation_msg.whycon_y=0.0
        self.biolocation_msg.whycon_z=0.0

        self.maxx_len=0

        self.cx=252.75
        self.cy=262.19
        self.fx=1459.52
        self.fy=1460.72
        self.camera_x=0.1
        self.camera_y=0.1

        self.len_cnts=0
        self.max_len=0
        self.clear_block=False
        self.list_alien=[0.5]
        self.align_block = True
        self.past_waypoint_idx =0
        self.landd=True
        self.buzzz_lock=True
        self.buzzzz_lock=True
        self.buz_lock=True
        self.buzzzzz_lock=True
        self.way_lock=True

        self.waypoint = [#[7.58,7.7,23],   ##BOTTOM RIGHT
                        [-7.5,-7.48,23],  ##TOP LEFT
                        #[7.78,-7.45,23],  ##TOP RIGHT
                        #[-7.55,7.85,23],   ##BOTTOM LEFT

                         #[5.7,7.68,23],
                         [3.75,7.85,23],
                         #[1.94,7.52,23],
                         [-0.05,7.85,23],
                         #[-1.98,7.5,23],
                         [-3.99,7.85,23],
                         [-5.8,7.85,23],
                         [-7.7,7.85,23],
                         [-7.7,5.6,23],
                         [-7.7,3.68,23],
                         #[-7.56,1.68,23],
                         [-7.7,-0.06,23],
                         #[-7.58,-1.9,23],
                         [-7.7,-3.85,23],
                         [-7.7,-5.82,23],
                         [-7.7,-7.7,23],
                         [-5.89,-7.7,23],
    
                         [-3.65,-7.7,23],
                         #[-2.04,-7.56,23],
                         [0.12,-7.7,23],
                         #[1.91,-7.5,23],
                         [3.99,-7.7,23],
                         [5.87,-7.7,23],

                         [7.78,-7.7,23],
                         [7.78,-5.57,23],
                         [7.78,-3.49,23],
                         #[7.68,-2.04,23],
                         [7.78,0.17,23],
                         #[7.68,2.24,23],
                         [7.78,3.9,23],


                         #[5.96,3.8,23],
                         [3.8,3.9,23],
                         #[1.88,3.8,23],
                         [-0.02,3.9,23],
                         #[-1.95,3.72,23],
                         [-3.82,3.9,23],
                         #[-3.72,1.82,23],
                         [-3.82,0.00,23],
                         #[-3.78,-2,23],
                         [-3.82,-3.75,23],
                         #[-2.08,-3.76,23],

                         [0.09,-3.72,23],
                         #[2.12,-3.76,23],
                         [3.9,-3.68,23],
                         #[3.85,-1.84,23],
                         [3.8,0.07,23],
                         #[2.06,-0.11,23],
                         [0.04,0.04,23]]

        self.current_waypoint_indx=0
        self.setpoint = self.waypoint[self.current_waypoint_indx]

        self.Kp = [9.06,5.56,8.8] 
        self.Kd = [467.1,586.5,348.5] 
        self.Ki = [0.075,0.0066,0.0298] 

        self.Base_z=0
        self.Base_x=0
        self.Base_y=0
        self.earliest_z = None
        self.hover_start = 0
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)       
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_roll= node.create_subscription(PidTune,"/pid_tuning_roll",self.pid_tune_roll_callback,1)
        self.pid_pitch = node.create_subscription(PidTune,"/pid_tuning_pitch",self.pid_tune_pitch_callback,1)
        self.biolocation_publisher = node.create_publisher(Biolocation,'/astrobiolocation',10)
        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.rc_unfilter = node.create_publisher(RCMessage, "/swift/rc_unfilter",1)        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)    
        self.limit = node.create_publisher(PIDError,"/limits",1)    
        self.image_sub =node.create_subscription(Image, '/video_frames', self.image_callback,10)
    ''' 
        * Function Name: whycon_poses_callback
        * Input: self.drone_whycon_pose_array.poses[0].position.x,
                 self.drone_whycon_pose_array.poses[0].position.y,
                 self.drone_whycon_pose_array.poses[0].position.z
        * Output: self.drone_position
        * Logic: Callback function for handling whycon poses messages. It updates the position of the drone based on whycon pose data.
        * Example Call: whycon_poses_callback(self, msg)
    '''
    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg
        self.drone_position[0] = self.drone_whycon_pose_array.poses[0].position.x
        self.drone_position[1] = self.drone_whycon_pose_array.poses[0].position.y
        self.drone_position[2] = self.drone_whycon_pose_array.poses[0].position.z
        self.whycon_data_recieved = True
        if self.earliest_z is None:
                if self.drone_whycon_pose_array.poses[0].position.z:
                    self.Base_z = self.drone_whycon_pose_array.poses[0].position.z 
                    self.Base_x=self.drone_whycon_pose_array.poses[0].position.x
                    self.Base_y=self.drone_whycon_pose_array.poses[0].position.y 
                    self.node.get_logger().info(f"BASE STATION : {self.Base_z}")
                    self.earliest_z = 1
    ''' 
        * Function Name: pid_tune_throttle_callback
        * Input: msg
        * Output: self.Kp,self.Ki,self.Kd
        * Logic: Callback function for handling PID tuning messages related to altitude. It updates the PID parameters for altitude control.
        * Example Call: pid_tune_throttle_callback(self, msg)
    '''
    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.05
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.5
    ''' 
        * Function Name: pid_tune_roll_callback
        * Input: msg
        * Output: self.Kp,self.Ki,self.Kd
        * Logic: Callback function for handling PID tuning messages related to roll. It updates the PID parameters for roll control.
        * Example Call: pid_tune_roll_callback(self, msg)
    '''
    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.kp * 0.01
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1
    ''' 
        * Function Name: pid_tune_pitch_callback
        * Input: msg
        * Output: self.Kp,self.Ki,self.Kd
        * Logic: Callback function for handling PID tuning messages related to pitch. It updates the PID parameters for pitch control.
        * Example Call: pid_tune_pitch_callback(self, msg)
    '''
    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1
    ''' 
        * Function Name: at_setpoint
        * Input: self.drone_position,self.setpoint
        * Output: x_at_setpoint and y_at_setpoint and z_at_setpoint
        * Logic: Checks if the drone has reached its setpoint position.
        * Example Call: self.at_setpoint()
    '''
    def at_setpoint(self):
        x_tolerance = 0.6
        y_tolerance = 0.6
        z_tolerance = 0.6
        x_at_setpoint = abs((self.setpoint[0]) - self.drone_position[0]) <= x_tolerance 
        y_at_setpoint = abs((self.setpoint[1]) - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs((self.setpoint[2]) - self.drone_position[2]) <= z_tolerance        
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    ''' 
        * Function Name: nextt
        * Input: self.rc_message
        * Output: self.rc_pub
        * Logic: Used for landing of the drone after a specific setpoint is reached.
        * Example Call: self.nextt()
    '''
    def nextt(self):
        self.node.get_logger().info("RETURNING TO BASE")
        self.land_time=time.time()
        current_setpoint =25
        while True:
            self.rc_pub.publish(self.rc_message)
            self.setpoint=[7,9,current_setpoint]
            while not self.at_setpoint():
                if time.time()-self.land_time >2:
                    self.disarm()
                pos=self.drone_whycon_pose_array.poses[0].position.z 
                print(pos, self.setpoint)
                self.pid()
            current_setpoint+=0.5
    ''' 
        * Function Name: image_callback
        * Input: image
        * Output: self.centroid_list,self.len_cnts,average_pixel_x,average_pixel_y,self.camera_x,self.camera_y
        * Logic: Callback function for handling image messages. It processes the image data to detect objects and perform tasks like alignment.
        * Example Call: self.image_callback(data)
    '''
    def image_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)
        labels = measure.label(thresh, connectivity=2, background=0)
        mask = np.zeros(thresh.shape, dtype="uint8")
        for label in np.unique(labels):
            if label == 0:
                continue
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255
            numPixels = cv2.countNonZero(labelMask)
            if numPixels > 300:
                mask = cv2.add(mask, labelMask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        centroid_list = []
        area_list = []
        for i, c in enumerate(cnts):
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                centroid = (cx, cy)
            else:
                centroid = (0.0, 0.0)
            centroid_list.append(centroid)
            area_list.append(area)
            self.centroid_list=centroid_list
            self.len_cnts=len(cnts)

            print(len(self.list_alien))
            
        if len(cnts) in [2,3,4,5] and self.clear_block==False and self.align_block:
            if len(cnts) not in self.list_alien:
                    self.in_hover_mode = True
                    self.hover_start_time = time.time()
                    if time.time()-self.in_hover_mode >=1:
                        self.in_hover_mode=False
                        self.aligning()
        
        self.max_len=self.len_cnts
        sum_pixel_x = 0
        sum_pixel_y = 0

        for cc in self.centroid_list:
            pixel_x,pixel_y=cc
            sum_pixel_x += pixel_x
            sum_pixel_y += pixel_y
        
        if len(self.centroid_list)>0:
            average_pixel_x = sum_pixel_x / len(self.centroid_list)
            average_pixel_y = sum_pixel_y / len(self.centroid_list)
            self.camera_x = (average_pixel_x - self.cx) / self.fx
            self.camera_y = (average_pixel_y - self.cy) / self.fy

    ''' 
        * Function Name: aligning
        * Input: self.drone_position,self.camera_x,self.maxx_len,self.align_block,self.biolocation_msg
        * Output: self.buz_lock,self.buzzz_lock,self.buzzzz_lock,self.buzzzzz_lock,self.list_alien
        * Logic: FUnction to align the drone to the centroid of the led.
        * Example Call: self.aligning()
    '''
    def aligning(self):
            if abs(self.camera_x) > 0.005 and abs(self.camera_y) > 0.005:
                self.setpoint[0]=(self.drone_position[0]+self.camera_x)
                self.setpoint[1]=(self.drone_position[1]+self.camera_y)
            else:
                self.align_block = False
                if self.len_cnts>self.maxx_len:
                    self.maxx_len=self.len_cnts
                if self.maxx_len==2:
                    self.alien_type='alien_a'
                elif self.maxx_len==3:
                    self.alien_type='alien_b'
                elif self.maxx_len==4:
                    self.alien_type='alien_c'
                elif self.maxx_len==5:
                    self.alien_type='alien_d'

                self.biolocation_msg.organism_type = self.alien_type  
                self.biolocation_msg.whycon_x = self.setpoint[0]
                self.biolocation_msg.whycon_y = self.setpoint[1]
                self.biolocation_msg.whycon_z = self.Base_z
                self.biolocation_publisher.publish(self.biolocation_msg)
        
                if self.alien_type == 'alien_a':
                    self.current_tim = time.time()
                    self.rc_message.aux3 = 2000
                    self.rc_message.aux4 = 1500
                    self.rc_pub.publish(self.rc_message)
                    self.buz_lock=False
                if self.alien_type == 'alien_b':
                    self.current_tim = time.time()
                    self.rc_message.aux3 = 2000
                    self.rc_message.aux4 = 1500
                    self.rc_pub.publish(self.rc_message)
                    self.buzzzz_lock=False
                if self.alien_type == 'alien_c':
                    self.current_tim = time.time()
                    self.rc_message.aux3 = 2000
                    self.rc_message.aux4 = 1500
                    self.rc_pub.publish(self.rc_message)
                    self.buzzzzz_lock=False
                if self.alien_type == 'alien_d':
                    self.current_tim = time.time()
                    self.rc_message.aux3 = 2000
                    self.rc_message.aux4 = 1500
                    self.rc_pub.publish(self.rc_message)
                    self.buzzz_lock=False
                print(self.maxx_len,self.len_cnts)
                self.list_alien.append(self.maxx_len)
                self.maxx_len=0
                self.past_waypoint_idx=self.current_waypoint_indx                
                print('aligned')

    ''' 
        * Function Name: next
        * Input: None
        * Output: self.waypoint,self.clear_block
        * Logic: Function used for setting the path used for safe landing the drone at the bottom right corner of the arena.
        * Example Call: self.next()
    '''
    def next(self):
        self.clear_block=True
        self.current_waypoint_indx=0
        self.landd=False
        self.waypoint=[[0,0,23],[3.78,3.84,25],[7.3,7.75,25],[7,9,25.5]]
        self.setpoint=self.waypoint[self.current_waypoint_indx]
        self.pid()

    ''' 
        * Function Name: pid
        * Input: self.way_lock,self.Base_x,self.Base_y,self.buz_lock,self.buzzz_lock,self.buzzzz_lock,self.buzzzzz_lock,
                self.hover_duration,self.hover_start_time,self.Kp,self.Ki,self.Kd,MIN_ROLL,MAX_ROLL,MIN_PITCH,MAX_PITCH,MIN_THROTTLE,MAX_THROTTLE,SUM_ERROR_ROLL_LIMIT,
                SUM_ERROR_PITCH_LIMIT,SUM_ERROR_THROTTLE_LIMIT,MAX_ROLL_FILTER,MIN_ROLL_FILTER,MAX_PITCH_FILTER,
                MIN_PITCH_FILTER,MAX_THROTTLE_FILTER,MIN_THROTTLE_FILTER,DRONE_WHYCON_POSE,
        * Output: self.error,self.rc_message,
        * Logic: This function is executed in loop to run the PID controller used to control the drone.
        * Example Call: self.pid()
    '''
                                       # PID algorithm
    def pid(self):    
        if 0>self.Base_x and self.way_lock:
            if 0>self.Base_y:
                self.waypoint=[[-7.7,-7.7,23],
                         [-5.89,-7.7,23],
    
                         [-3.65,-7.7,23],
                         #[-2.04,-7.56,23],
                         [0.12,-7.7,23],
                         #[1.91,-7.5,23],
                         [3.99,-7.7,23],
                         [5.87,-7.7,23],

                         [7.78,-7.7,23],
                         [7.78,-5.57,23],
                         [7.78,-3.49,23],
                         #[7.68,-2.04,23],
                         [7.78,0.17,23],
                         #[7.68,2.24,23],
                         [7.78,3.9,23],

                         [7.78,5.88,23],###
                         [7.78,7.85,23],
                         [5.7,7.85,23],###
                         [3.75,7.85,23],
                         #[1.94,7.52,23],
                         [-0.05,7.85,23],
                         #[-1.98,7.5,23],
                         [-3.99,7.85,23],
                         [-5.8,7.85,23],
                         [-7.7,7.85,23],
                         [-7.7,5.6,23],
                         [-7.7,3.68,23],
                         #[-7.56,1.68,23],
                         [-7.7,-0.06,23],
                         #[-7.58,-1.9,23],
                         [-7.7,-3.85,23],


                         #[-5.65,-3.7,23],
                         [-3.82,-3.75,23],
                         #[-2.08,-3.76,23],

                         [0.09,-3.72,23],
                         #[2.12,-3.76,23],
                         [3.9,-3.68,23],
                         #[3.85,-1.84,23],
                         [3.8,0.07,23],

                         [3.8,3.79,23],
                         [-0.04,3.79,23],

                         #[-1.95,3.72,23],
                         [-3.82,3.7,23],
                         #[-3.72,1.82,23],
                         [-3.82,-0.13,23],
                         #[-1.88,-0.04,23],
                         [0.04,-0.02,23]]
                self.way_lock=False

        if 0<self.Base_x and self.way_lock:
            if 0>self.Base_y:
                self.waypoint=[[7.78,-7.7,23],
                         [7.78,-5.57,23],
                         [7.78,-3.49,23],
                         #[7.68,-2.04,23],
                         [7.78,0.17,23],
                         #[7.68,2.24,23],
                         [7.78,3.9,23],

                         [7.78,5.88,23],###
                         [7.78,7.85,23],
                         [5.7,7.85,23],###
                         [3.75,7.85,23],
                         #[1.94,7.52,23],
                         [-0.05,7.85,23],
                         #[-1.98,7.5,23],
                         [-3.99,7.85,23],
                         [-5.8,7.85,23],
                         [-7.7,7.85,23],
                         [-7.7,5.6,23],
                         [-7.7,3.68,23],
                         #[-7.56,1.68,23],
                         [-7.7,-0.06,23],
                         #[-7.58,-1.9,23],
                         [-7.7,-3.85,23],

                         #[-7.56,-5.82,23],
                         [-7.7,-7.7,23],
                         [-5.89,-7.7,23],
    
                         [-3.65,-7.7,23],
                         #[-2.04,-7.56,23],
                         [0.12,-7.7,23],
                         #[1.91,-7.5,23],
                         [3.99,-7.7,23],

                       #[3.87,-5.45,23],
                       [3.9,-3.68,23],
                         #[3.85,-1.84,23],
                         [3.8,0.07,23],

                         [3.8,3.79,23],
                         [-0.04,3.79,23],

                         #[-1.95,3.72,23],
                         [-3.7,3.79,23],
                         #[-3.72,1.82,23],
                         [-3.7,-0.07,23],
                         #[-3.78,-2,23],
                         [-3.7,-3.79,23],
                         #[-2.08,-3.76,23],

                         [0.12,-3.79,23],

                         #[0.04,-1.96,23],
                         [0.03,0.01,23]]
                self.way_lock=False
               
        if 0>self.Base_x and self.way_lock:
            if 0<self.Base_y:
                self.waypoint=[[-7.7,7.85,23],
                         [-7.7,5.6,23],
                         [-7.7,3.68,23],
                         #[-7.56,1.68,23],
                         [-7.7,-0.06,23],
                         #[-7.58,-1.9,23],
                         [-7.7,-3.85,23],
                         [-7.7,-5.82,23],
                         [-7.7,-7.7,23],
                         [-5.89,-7.7,23],
    
                         [-3.65,-7.7,23],
                         #[-2.04,-7.56,23],
                         [0.12,-7.7,23],
                         #[1.91,-7.5,23],
                         [3.99,-7.7,23],
                         [5.87,-7.7,23],

                         [7.78,-7.7,23],
                         [7.78,-5.57,23],
                         [7.78,-3.49,23],
                         #[7.68,-2.04,23],
                         [7.78,0.17,23],
                         #[7.68,2.24,23],
                         [7.78,3.9,23],

                         [7.78,5.88,23],###
                         [7.78,7.85,23],
                         [5.7,7.85,23],###
                         [3.75,7.85,23],
                         #[1.94,7.52,23],
                         [-0.05,7.85,23],
                         #[-1.98,7.5,23],
                         [-3.99,7.85,23],

                         [-3.82,3.9,23],
                         #[-3.72,1.82,23],
                         [-3.82,0.00,23],
                         #[-3.78,-2,23],
                         [-3.82,-3.75,23],
                         #[-2.08,-3.76,23],

                         [0.09,-3.72,23],
                         #[2.12,-3.76,23],
                         [3.9,-3.68,23],
                         #[3.85,-1.84,23],
                         [3.8,0.07,23],

                         [3.8,3.79,23],
                         [-0.04,3.79,23],
                         [0.00,-0.13,23]]
                self.way_lock=False

        if not self.buz_lock:
            if time.time()-self.current_tim>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.3:
                self.rc_message.aux3 = 1000
        if not self.buz_lock:
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=0.6:
                self.rc_message.aux4 = 1000
        if not self.buzzzz_lock:
            if time.time()-self.current_tim>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.3:
                self.rc_message.aux3 = 1000
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.5:
                self.rc_message.aux3 = 1000
        if not self.buzzzz_lock:
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=0.6:
                self.rc_message.aux4 = 1000
            if time.time()-self.current_tim>=0.8:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=1:
                self.rc_message.aux4 = 1000
        if not self.buzzzzz_lock:
            if time.time()-self.current_tim>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.3:
                self.rc_message.aux3 = 1000
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.5:
                self.rc_message.aux3 = 1000
            if time.time()-self.current_tim>=0.6:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.7:
                self.rc_message.aux3 = 1000
        if not self.buzzzzz_lock:
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=0.6:
                self.rc_message.aux4 = 1000
            if time.time()-self.current_tim>=0.8:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=1:
                self.rc_message.aux4 = 1000
            if time.time()-self.current_tim>=1.2:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=1.4:
                self.rc_message.aux4 = 1000
        if not self.buzzz_lock:
            if time.time()-self.current_tim>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.3:
                self.rc_message.aux3 = 1000
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.6:
                self.rc_message.aux3 = 1000
            if time.time()-self.current_tim>=0.7:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=0.8:
                self.rc_message.aux3 = 1000
            if time.time()-self.current_tim>=0.9:
                self.rc_message.aux3 = 2000
            if time.time()-self.current_tim>=1:
                self.rc_message.aux3 = 1000
        if not self.buzzz_lock:
            if time.time()-self.current_tim>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.current_tim>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=0.6:
                self.rc_message.aux4 = 1000
            if time.time()-self.current_tim>=0.8:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=1:
                self.rc_message.aux4 = 1000
            if time.time()-self.current_tim>=1.2:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=1.4:
                self.rc_message.aux4 = 1000
            if time.time()-self.current_tim>=1.6:
                self.rc_message.aux4 = 1500
            if time.time()-self.current_tim>=1.8:
                self.rc_message.aux4 = 1000

        print("INSIDE PID LOOOOP")
        print(self.past_waypoint_idx)
        print(self.current_waypoint_indx)
        print(self.camera_x,self.camera_y)
        print(self.align_block)
    
        if self.current_waypoint_indx < len(self.waypoint):
            if self.at_setpoint() and not self.in_hover_mode:
                self.hover_start_time = time.time()
                self.in_hover_mode = True
            elif self.in_hover_mode and (time.time() - self.hover_start_time) >= self.hover_duration:
                self.in_hover_mode = False
                self.current_waypoint_indx += 1
                if self.current_waypoint_indx < len(self.waypoint):
                    self.setpoint = self.waypoint[self.current_waypoint_indx]
        # 0 : calculating Error, Derivative, Integral for Roll error : x axis
        try:
            if self.current_waypoint_indx >= self.past_waypoint_idx+3:
                print("ALIEN_BLOCK")
                self.align_block=True

            self.error[0] = -(self.drone_whycon_pose_array.poses[0].position.x - self.setpoint[0] )
            self.error[1] = (self.drone_whycon_pose_array.poses[0].position.y - self.setpoint[1] )
            self.error[2] = -(self.setpoint[2] - self.drone_whycon_pose_array.poses[0].position.z) 
        except:
            pass
        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)
        print(self.rc_message.rc_throttle)

        self.proportional[0] = (self.Kp[0] * self.error[0])
        self.integral[0] = self.sum_error[0]*self.Ki[0]
        self.derrivative[0] = (self.prev_error[0] - self.error[0])*self.Kd[0]

        self.proportional[1] = (self.Kp[1] * self.error[1])
        self.integral[1] = self.sum_error[1]*self.Ki[1]
        self.derrivative[1] = (self.prev_error[1] - self.error[1])*self.Kd[1]

        self.proportional[2] = (self.Kp[2] * self.error[2])
        self.integral[2] = self.sum_error[2]*self.Ki[2]
        self.derrivative[2] = ((self.prev_error[2] - self.error[2]))*self.Kd[2]
        
        self.sum_error[0]=self.sum_error[0] + self.error[0]
        self.sum_error[1]=self.sum_error[1] + self.error[1]
        self.sum_error[2]=self.sum_error[2] + self.error[2]

        if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
            self.integral[0] = SUM_ERROR_ROLL_LIMIT
        if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
            self.integral[0] = -SUM_ERROR_ROLL_LIMIT

        if self.integral[1] > SUM_ERROR_PITCH_LIMIT:
            self.integral[1] = SUM_ERROR_PITCH_LIMIT
        if self.integral[1] < -SUM_ERROR_PITCH_LIMIT:
            self.integral[1] = -SUM_ERROR_PITCH_LIMIT

        if self.integral[2] > SUM_ERROR_THROTTLE_LIMIT:
            self.integral[2] = SUM_ERROR_THROTTLE_LIMIT
        if self.integral[2] < -SUM_ERROR_THROTTLE_LIMIT:
            self.integral[2] = -SUM_ERROR_THROTTLE_LIMIT

        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis
        self.rc_throttle = 1450 +int(self.proportional[2] + self.integral[2] - self.derrivative[2])
        self.rc_pitch = 1492.5 +int(self.proportional[1] + self.integral[1] - self.derrivative[1])
        self.rc_roll = 1502.5 +int(self.proportional[0] + self.integral[0] - self.derrivative[0])

    #------------------------------------------------------------------------------------------------------------------------
        
        throttle = self.rc_throttle
        roll = self.rc_roll
        pitch = self.rc_pitch

        self.prev_error = copy.deepcopy(self.error)
        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.5,
                zero_error=0.0,
            )
        )

        self.limit.publish(
            PIDError(
                yaw_error = 0.5
            )
        )

        self.rc_message.rc_yaw = int(1500)

        if roll > MAX_ROLL_FILTER:     #checking range i.e. bet 1000 and 2000
            roll = MAX_ROLL_FILTER
        elif roll < MIN_ROLL_FILTER:
            roll = MIN_ROLL_FILTER
        
        if pitch > MAX_PITCH_FILTER:     #checking range i.e. bet 1000 and 2000
            pitch = MAX_PITCH_FILTER
        elif pitch < MIN_PITCH_FILTER:
            pitch = MIN_PITCH_FILTER

        if throttle > MAX_THROTTLE_FILTER:     #checking range i.e. bet 1000 and 2000
            throttle = MAX_THROTTLE_FILTER
        elif throttle < MIN_THROTTLE_FILTER:
            throttle = MIN_THROTTLE_FILTER

                                 # BUTTERWORTH FILTER
        
        span = 15

        for index, val in enumerate([roll, pitch, throttle]):
            DRONE_WHYCON_POSE[index].append(val)
            if len(DRONE_WHYCON_POSE[index]) == span:
                DRONE_WHYCON_POSE[index].pop(0)
            if len(DRONE_WHYCON_POSE[index]) != span-1:
                return
            order = 3
            fs = 30
            fc = 5
            nyq = 0.5 * fs
            wc = fc / nyq
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])

            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])

            if index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])

        if self.rc_message.rc_roll > MAX_ROLL:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL
        
        if self.rc_message.rc_pitch > MAX_PITCH:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH

        if self.rc_message.rc_throttle > MAX_THROTTLE:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE

        self.rc_msg.rc_roll = int(roll)
        self.rc_msg.rc_pitch = int(pitch)
        self.rc_msg.rc_throttle = int(throttle)
        
        self.rc_pub.publish(self.rc_message)
        self.rc_unfilter.publish(self.rc_msg)
        print(self.hover_duration)

        if len(self.list_alien)==3 and self.landd:
            self.next()

        if self.setpoint == [7,9,25.5]:
            if self.at_setpoint():
                print("LANDING FOR FINAL SETPOINT")
                self.now()
    
        print(self.setpoint)   
        print(self.list_alien)     

    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 
    ''' 
        * Function Name: shutdown_hook
        * Input: None
        * Output: None
        * Logic: This function is used to call the disarm function.
        * Example Call: self.shutdown_hook()
    '''
    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()
    ''' 
        * Function Name: arm
        * Input: None
        * Output: None
        * Logic: This function is used to arm the drone .
        * Example Call: self.arm()
    '''
    # Function to arm the drone 
    def arm(self):
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)
    # Function to disarm the drone 
    ''' 
        * Function Name: disarm
        * Input: None
        * Output: None
        * Logic: This function is used to disarm the drone .
        * Example Call: self.disarm()
    '''
    def disarm(self):
        self.rc_message.rc_roll=MIN_ROLL
        self.rc_message.rc_pitch=MIN_PITCH
        self.rc_message.rc_throttle=MIN_THROTTLE
        self.node.get_logger().info("Calling DISARM service")
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)
        # Create the disarm function
    ''' 
        * Function Name: now
        * Input: None
        * Output: self.nextt()
        * Logic: This function is used to prepare the drone for the landing function.
        * Example Call: self.now()
    '''
    def now(self):
        self.node.get_logger().info("REACHED THE INITIAL HEIGHT : HOVERING STARTED")
        self.rc_throttle = 1450
        self.hover_start = time.time()
        self.timer = False
        if time.time() - self.hover_start > 3:       
            self.nextt()
        else:
            pass
''' 
        * Function Name: main
        * Input: DroneCOntroller
        * Output: None
        * Logic: This is the main function of the code which is used to initialize the drone controller.
        * Example Call: main()
'''
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('controller')
    node.get_logger().info(f"Node Started")
    node.get_logger().info("Entering PID controller loop")
    controller = DroneController(node)
    controller.arm()
    node.get_logger().info("Armed")
    desired_rate = 50.0
    timer_period = rclpy.duration.Duration(seconds=1.0/desired_rate)
    timer = node.create_timer(timer_period.nanoseconds*1e-9, controller.pid)
    try:
        while rclpy.ok():
                if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 10:
                    node.get_logger().error("Unable to detect WHYCON poses")
                rclpy.spin_once(node) # Sleep for 1/30 secs
    except Exception as err:
        print(err)
    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()