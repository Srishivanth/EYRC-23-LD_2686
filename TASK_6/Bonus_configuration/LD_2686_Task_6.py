#!/usr/bin/env python3

"""
Controller for the drone
* Team Id : 2686
* Author List : Srishivanth R F, Krishna Prasath S, Shyam M, Joseph Raj B
* Filename: LD_2686_Task_6
* Theme: Luminosity Drone
* Functions: DroneController,whycon_poses_callback,pid_tune_throttle_callback,pid_tune_roll_callback,
            pid_tune_pitch_callback,at_setpoint,landing,image_callback,aligning,landing_prep,pid,shutdown_hook,
            arm,disarm,start_landing,main
* Global Variables: ##self.timer,self.rc_message.aux1,self.rc_message.aux2,self.rc_message.aux3,
        self.rc_message.aux4,self.hover_duration,self.in_hover_mode,self.alien_type,
        self.biolocation_msg.organism_type,self.biolocation_msg.whycon_x,self.biolocation_msg.whycon_y,
        self.biolocation_msg.whycon_z,self.maxx_len,self.cx,self.cy,self.fx,self.fy,self.camera_x,
        self.camera_y,self.len_cnts,self.max_len,self.clear_block,self.list_alien,self.avoid_aligning,
        self.detected_waypoint_index,self.started_landing,self.d_detected,self.b_detected,self.a_detected,self.c_detected,
        self.waypoint_decided,self.waypoint,self.current_waypoint_indx,self.takeoff_z,self.takeoff_x,self.takeoff_y,
        self.first_whycon,self.node,self.rc_message,self.rc_msg,self.drone_whycon_pose_array,
        self.last_whycon_pose_received_at,self.commandbool,service_endpoint,
        self.bridge,self.centroid_list,self.aligned_time,self.cu_time,self.rc_throttle,
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


#The below variables are used to set the limits for the output of the Butterworth filter
MIN_ROLL = 1400
MAX_ROLL = 1600

MIN_PITCH = 1400
MAX_PITCH = 1600

MIN_THROTTLE = 1400
MAX_THROTTLE = 1600

#The below variables are used to set the limits for the input of the Butterworth filter
MAX_ROLL_FILTER = 2000
MIN_ROLL_FILTER = 1000

MAX_PITCH_FILTER = 2000
MIN_PITCH_FILTER = 1000

MAX_THROTTLE_FILTER = 2000
MIN_THROTTLE_FILTER = 1000

#The below variables are used for setting the limit for the integral sum error for pitch,roll and throttle
SUM_ERROR_ROLL_LIMIT = 4
SUM_ERROR_PITCH_LIMIT = 250
SUM_ERROR_THROTTLE_LIMIT = 10000

DRONE_WHYCON_POSE = [[], [], []]   #variable used for the filtering of the rc_command

''' 
        * Function Name: whycon_poses_callback
        * Input: None
        * Output: #self.timer,self.rc_message.aux1,self.rc_message.aux2,self.rc_message.aux3,
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
        self.commandbool = CommandBool.Request()
        service_endpoint = "/swift/cmd/arming"
        self.bridge=CvBridge()
        self.centroid_list=[]

        self.arming_service_client = self.node.create_client(CommandBool,service_endpoint)

        #These are some lists used to store the PID calculation variables and values for X, Y and Z axes
        self.error = [0.0, 0.0, 0.0]               
        self.integral = [0.0,0.0,0.0]
        self.proportional = [0.0,0.0,0.0]
        self.derrivative = [0.0,0.0,0.0]
        self.prev_error = [0.0,0.0,0.0]
        self.sum_error=[0,0,0]
        self.drone_position =[0,0,0]

        #These are the Proportional, Derrivative and Integral constant values for the PID controller of the drone
        self.Kp = [9.06,5.56,8.8] 
        self.Kd = [467.1,586.5,348.5] 
        self.Ki = [0.075,0.0066,0.0298] 

        #These are the initial values assigned for the AUX1, AUX2, AUX3 and AUX4 message
        self.rc_message.aux1 = 1500
        self.rc_message.aux2 = 1500
        self.rc_message.aux3 = 1000
        self.rc_message.aux4 = 1000
        
        #The below variables are used to initialize the type for the biolocation message used to publish alien type and coordinates
        self.alien_type=''
        self.biolocation_msg=Biolocation()
        self.biolocation_msg.organism_type=''
        self.biolocation_msg.whycon_x=0.0
        self.biolocation_msg.whycon_y=0.0
        self.biolocation_msg.whycon_z=0.0

        self.hover_duration = 0.5  #variable used for storing the time the drone has to stablize or hover on each waypoint
        self.in_hover_mode = False  #variable used to indicate and check if the drone is in hovering state or not

        #These are the intrinsic parameters of the camera module "Raspberry Pi Camera V2" found by ROS camera caliberation method
        #These parameters are used to convert the pixel coordinates of the LED cluster into Whycon coordinates
        self.cx=252.75  #x-axis focal length of camera in pixels
        self.cy=262.19  #y-axis focal length of camera in pixels
        self.fx=1459.52 #x-axis optical center of camera in pixels
        self.fy=1460.72 #y-axis optical center of camera in pixels

        #The below variables are used to store the difference in whycon coordinates that the drone need to overcome for aligning directly above the LED
        self.camera_x=0.1
        self.camera_y=0.1   #Initialized as 0.1 and not 0 to avoid the case where the drone doesnot align with the LED becoz the difference would be 0 meaning the drone is aligned.


        self.len_cnts=0       #Variable used to store the count of contours detected in the current callback.
        self.avoid_detect=False    #This FLAG variable is used to avoid detection of LED after all the led's are detected and the drone is following the landing procedure.
        self.list_alien=[0]     #This is the list used for storing the number of LED of various clusters detected, aligned and published
        self.avoid_aligning = False     #This is a FLAG variable used for avoiding the drone aligning to the same LED more than once at the same time
           
        self.started_landing=False     #This is a FLAG variable used for avoiding the repetitive assignment of the same landing setpoint and enabling successful landing.

        #The below four variables are used as a FLAG variable to indicate which LED cluster/alien has been currently detected for blinking of LED and beep of buzzer.
        self.d_detected=False
        self.b_detected=False
        self.a_detected=False
        self.c_detected=False

        self.current_waypoint_indx=0       #This variable is used for storing the search waypoint index for acessing each setpoint



        self.waypoint_decided=False     #This FLAG variable is used to decide the search waypoint that is to be used depending on takeoff point

        self.timer=True


        

        self.detected_waypoint_index =0    #This variable is used to store the waypoint index at which the LED was detcted in order to keep track of the point at which LED was detected.
        
        
        self.first_whycon_recieved = False     #This FLAG variable is used to check if the code has recieved it's first whycon coordinates for storing that coordinate in another variable

        #The below three variables are used to store the takeoff coordinate for deciding the search algorithm.
        self.takeoff_x=0
        self.takeoff_y=0
        self.takeoff_z=0
        
        #SUBSCRIBING TO THE REQUIRED TOPICS 
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)       
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_roll= node.create_subscription(PidTune,"/pid_tuning_roll",self.pid_tune_roll_callback,1)
        self.pid_pitch = node.create_subscription(PidTune,"/pid_tuning_pitch",self.pid_tune_pitch_callback,1)
        self.image_sub =node.create_subscription(Image, '/video_frames', self.image_callback,10)

        #PUBLISHING THE NECESSARY INFORMATION
        self.biolocation_publisher = node.create_publisher(Biolocation,'/astrobiolocation',10)
        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.rc_unfilter = node.create_publisher(RCMessage, "/swift/rc_unfilter",1)        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)    
        self.limit = node.create_publisher(PIDError,"/limits",1)    
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
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]    #Used to determine if whycon coordinates are available using 'if' statement in the main block

        #storing the values in a list called drone_position for easy access
        self.drone_whycon_pose_array = msg
        self.drone_position[0] = self.drone_whycon_pose_array.poses[0].position.x
        self.drone_position[1] = self.drone_whycon_pose_array.poses[0].position.y
        self.drone_position[2] = self.drone_whycon_pose_array.poses[0].position.z

        #This code is used to get the First ever whycon coordinate after running the code for determining the takeoff point.
        if not self.first_whycon_recieved:
                if self.drone_whycon_pose_array.poses[0].position.z:
                    self.takeoff_x=self.drone_whycon_pose_array.poses[0].position.x
                    self.takeoff_y=self.drone_whycon_pose_array.poses[0].position.y 
                    self.takeoff_z=self.drone_whycon_pose_array.poses[0].position.z 
                    self.node.get_logger().info(f"BASE STATION : {self.takeoff_z}")
                    self.first_whycon_recieved = True       #Make this variable false to ensure the 'if' part is run only once and only the initial whycon is stored.
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
        * Output: (x_at_setpoint and y_at_setpoint and z_at_setpoint)
        * Logic: Checks if the drone has reached its setpoint position.
        * Example Call: self.at_setpoint()
    '''
    def at_setpoint(self):

        #This code is used to check if the drone is inside the error range box of 0.6 and returns True if it is inside.
        x_tolerance = 0.8
        y_tolerance = 0.8
        z_tolerance = 0.8
        x_at_setpoint = abs((self.setpoint[0]) - self.drone_position[0]) <= x_tolerance 
        y_at_setpoint = abs((self.setpoint[1]) - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs((self.setpoint[2]) - self.drone_position[2]) <= z_tolerance        
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    
    ''' 
        * Function Name: start_landing
        * Input: None
        * Output: self.landing()
        * Logic: This function is used to prepare the drone for the landing function.
        * Example Call: self.start_landing()
    '''
    def start_landing(self):
        self.rc_throttle = 1450
        if self.timer :
            self.hover_start = time.time()
            self.timer = False
            #print("before 10 secs")
        
        if time.time() - self.hover_start > 1:       
            #print("after 10 secs")     
            self.landing()
        else:
            pass

    ''' 
        * Function Name: landing
        * Input: self.rc_message
        * Output: self.rc_pub
        * Logic: Used for landing of the drone after a specific setpoint is reached.
        * Example Call: self.landing()
    '''
    def landing(self):
        self.node.get_logger().info("RETURNING TO BASE")
        start_landing_time=time.time()
        current_setpoint =25
        while True:
            self.rc_pub.publish(self.rc_message)
            self.setpoint=[7,9,current_setpoint]
            while not self.at_setpoint():
                if time.time()-start_landing_time >2:
                    self.disarm()
                pos=self.drone_whycon_pose_array.poses[0].position.z 
                print(pos, self.setpoint)
                self.pid()
            current_setpoint+=0.5
    ''' 
        * Function Name: image_callback
        * Input: image
        * Output: self.centroid_list,self.len_cnts,average_pixel_x,average_pixel_y,self.camera_x,self.camera_y
        * Logic: Callback function for handling image messages. It processes the image data to detect LED and perform calculations for tasks like alignment.
        * Example Call: self.image_callback(data)
    '''
    def image_callback(self, data):

        # Convert ROS image message to OpenCV image
        image = self.bridge.imgmsg_to_cv2(data, data.encoding)

        # Convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur to the grayscale image
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        # Apply binary thresholding to obtain a binary image
        thresh = cv2.threshold(blurred, 100, 255, cv2.THRESH_BINARY)[1]

        # Erode and dilate the binary image to remove noise and close gaps
        thresh = cv2.erode(thresh, None, iterations=2)
        thresh = cv2.dilate(thresh, None, iterations=4)

        # Label connected components in the binary image
        labels = measure.label(thresh, connectivity=2, background=0)

        # Initialize an empty mask to store the segmented regions
        mask = np.zeros(thresh.shape, dtype="uint8")

        # Iterate over each label in the labeled image
        for label in np.unique(labels):
            # Skip background label (0)
            if label == 0:
                continue

            # Create a binary mask for the current label
            labelMask = np.zeros(thresh.shape, dtype="uint8")
            labelMask[labels == label] = 255

            # Count the number of non-zero pixels in the mask
            numPixels = cv2.countNonZero(labelMask)

            # If the number of pixels exceeds a threshold, add the mask to the final mask
            if numPixels > 300:
                mask = cv2.add(mask, labelMask)

        # Find contours in the final mask
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Initialize lists to store centroid coordinates and areas of contours
        centroid_list = []
        area_list = []

        # Iterate over each contour
        for i, c in enumerate(cnts):

            # Calculate the area of the contour
            area = cv2.contourArea(c)

            # Calculate the centroid of the contour
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx = M["m10"] / M["m00"]
                cy = M["m01"] / M["m00"]
                centroid = (cx, cy)
            else:
                centroid = (0.0, 0.0)

            # Append the centroid and area to their respective lists
            centroid_list.append(centroid)
            area_list.append(area)

            # Update class variables with centroid list and number of contours
            self.centroid_list=centroid_list
            self.len_cnts=len(cnts)

            print(len(self.list_alien))
        
        # Perform aligning if LED's 2,3,4 or 5 are detected and when anti-aligning FLAG variables are also in True state
        if len(cnts) in [2,3,4,5] and self.avoid_detect==False and not self.avoid_aligning:
            if len(cnts) not in self.list_alien:
                    self.in_hover_mode = True
                    self.hover_start_time = time.time()
                    if time.time()-self.in_hover_mode >=1:
                        self.in_hover_mode=False
                        self.aligning()

         # Calculate average centroid coordinates for clusters of more than one LED
        sum_pixel_x = 0
        sum_pixel_y = 0

        for cc in self.centroid_list:
            pixel_x,pixel_y=cc
            sum_pixel_x += pixel_x
            sum_pixel_y += pixel_y
        
        if len(self.centroid_list)>0:

            # Calculate camera coordinates based on centroid coordinates
            average_pixel_x = sum_pixel_x / len(self.centroid_list)
            average_pixel_y = sum_pixel_y / len(self.centroid_list)

            # Calculate difference in WHYCON coordinates based on camera coordinates
            self.camera_x = (average_pixel_x - self.cx) / self.fx
            self.camera_y = (average_pixel_y - self.cy) / self.fy

    ''' 
        * Function Name: aligning
        * Input: self.drone_position,self.camera_x,self.maxx_len,self.avoid_aligning,self.biolocation_msg
        * Output: self.a_detected,self.d_detected,self.b_detected,self.c_detected,self.list_alien
        * Logic: FUnction to align the drone to the centroid of the led.
        * Example Call: self.aligning()
    '''
    def aligning(self):
            
            #Check if the difference between the drone and the LED coordinates is greater than 0.005
            if abs(self.camera_x) > 0.005 and abs(self.camera_y) > 0.005:

                #Update the setpoint continuously and hence aligning to LED until the error range is very less
                self.setpoint[0]=(self.drone_position[0]+self.camera_x)
                self.setpoint[1]=(self.drone_position[1]+self.camera_y)
            else:

                #This FLAG variable is edited so that when once aligned the drone cannot align again to the same LED.
                self.avoid_aligning = True
               
               #According to the number of LED detected assign the alien type to the variable
                if self.len_cnts==2:
                    self.alien_type='alien_a'
                elif self.len_cnts==3:
                    self.alien_type='alien_b'
                elif self.len_cnts==4:
                    self.alien_type='alien_c'
                elif self.len_cnts==5:
                    self.alien_type='alien_d'

                #publish the alien type and whycon coordinates of the LED to the astrobiolocation topic
                self.biolocation_msg.organism_type = self.alien_type  
                self.biolocation_msg.whycon_x = self.setpoint[0]
                self.biolocation_msg.whycon_y = self.setpoint[1]
                self.biolocation_msg.whycon_z = self.takeoff_z
                self.biolocation_publisher.publish(self.biolocation_msg)
        
                #Switch the buzzer ON and make the LED of the drone RED and make it beep and blink later in pid loop
                self.rc_message.aux3 = 2000
                self.rc_message.aux4 = 1500
                self.rc_pub.publish(self.rc_message)
                self.aligned_time = time.time()

                #Make the particular FLAG variables of particular aliens True so that we can identify later which LED was detected and hence control the number of beeps and blinks we do
                if self.alien_type == 'alien_a':
                    self.a_detected=True
                if self.alien_type == 'alien_b':
                    self.b_detected=True
                if self.alien_type == 'alien_c':
                    self.c_detected=True
                if self.alien_type == 'alien_d':
                    self.d_detected=True

                #Append the number of LED to the list so that we can keep track of what aliens have been detected so that we can proceed to landing when necessary
                self.list_alien.append(self.len_cnts)

                #store the waypoint index at which the drone aligned and detected the alien in a variable so that we can temporarily disable the aligning part to avoid aligning to same LED twice
                self.detected_waypoint_index=self.current_waypoint_indx       
                print(self.len_cnts)         
                print('aligned')

    ''' 
        * Function Name: landing_prep
        * Input: None
        * Output: self.waypoint,self.clear_block
        * Logic: Function used for setting the path used for safe landing the drone at the bottom right corner of the arena.
        * Example Call: self.landing_prep()
    '''
    def landing_prep(self):

        #Turning this FLAG variable to True ensures in the IMAGE CALLBACK function that during landing even if the drone detects and LED it doesnot align itself to it and hence proceeds to safe landing
        self.avoid_detect=True
        self.current_waypoint_indx=0  #Reset the current waypoint index due to the introduction of a new landing waypoint
        self.started_landing=True      #This FLAG variable is used to ensure that the same waypoint is not set again and again as setpoint and hence staying in the first setpoint of the waypoint

        #Landing waypoint
        self.waypoint=[[0,0,23],[3.78,3.84,25],[7.3,7.75,25],[7,9,25.5]]
        self.setpoint=self.waypoint[self.current_waypoint_indx]
        self.pid()

    ''' 
        * Function Name: pid
        * Input: self.waypoint_decided,self.takeoff_x,self.takeoff_y,self.a_detected,self.d_detected,self.b_detected,self.c_detected,
                self.hover_duration,self.hover_start_time,self.Kp,self.Ki,self.Kd,MIN_ROLL,MAX_ROLL,MIN_PITCH,MAX_PITCH,MIN_THROTTLE,MAX_THROTTLE,SUM_ERROR_ROLL_LIMIT,
                SUM_ERROR_PITCH_LIMIT,SUM_ERROR_THROTTLE_LIMIT,MAX_ROLL_FILTER,MIN_ROLL_FILTER,MAX_PITCH_FILTER,
                MIN_PITCH_FILTER,MAX_THROTTLE_FILTER,MIN_THROTTLE_FILTER,DRONE_WHYCON_POSE,
        * Output: self.error,self.rc_message,
        * Logic: This function is executed in loop to run the PID controller used to control the drone.
        * Example Call: self.pid()
    '''
                                       # PID algorithm
    def pid(self):    


        # Deciding algorithm for the drone to set the path which needs to be followed for various takeoff points
        if 0>self.takeoff_x and not self.waypoint_decided:
            if 0>self.takeoff_y:
                self.waypoint = [
                                [-7.53, -8.21, 23.0],
                                [-7.49, -8.17, 23.0],
                                [-7.45, -8.12, 23.0],
                                [-7.41, -8.07, 23.0],
                                [-7.37, -8.03, 23.0],
                                [-5.46, -8.0, 23.0],
                                [-3.54, -7.98, 23.0],
                                [-1.63, -7.96, 23.0],
                                [0.28, -7.93, 23.0],
                                [1.23, -7.88, 23.0],
                                [2.19, -7.83, 23.0],
                                [3.14, -7.78, 23.0],
                                [4.1, -7.73, 23.0],
                                [5.07, -7.71, 23.0],
                                [6.04, -7.7, 23.0],
                                [7.0, -7.68, 23.0],
                                [7.97, -7.66, 23.0],
                                [7.92, -6.71, 23.0],
                                [7.88, -5.77, 23.0],
                                [7.83, -4.82, 23.0],
                                [7.78, -3.87, 23.0],
                                [7.77, -2.94, 23.0],
                                [7.76, -2.0, 23.0],
                                [7.75, -1.06, 23.0],
                                [7.74, -0.13, 23.0],
                                [7.71, 0.79, 23.0],
                                [7.68, 1.71, 23.0],
                                [7.65, 2.62, 23.0],
                                [7.62, 3.54, 23.0],
                                [7.63, 4.56, 23.0],
                                [7.65, 5.58, 23.0],
                                [7.67, 6.61, 23.0],
                                [7.68, 7.63, 23.0],
                                [6.74, 7.62, 23.0],
                                [5.8, 7.61, 23.0],
                                [4.86, 7.59, 23.0],
                                [3.92, 7.58, 23.0],
                                [2.93, 7.55, 23.0],
                                [1.94, 7.53, 23.0],
                                [0.94, 7.5, 23.0],
                                [-0.05, 7.47, 23.0],
                                [-0.99, 7.39, 23.0],
                                [-1.92, 7.32, 23.0],
                                [-2.85, 7.25, 23.0],
                                [-3.79, 7.17, 23.0],
                                [-4.78, 7.17, 23.0],
                                [-5.77, 7.17, 23.0],
                                [-6.76, 7.18, 23.0],
                                [-7.75, 7.18, 23.0],
                                [-7.71, 6.23, 23.0],
                                [-7.67, 5.29, 23.0],
                                [-7.63, 4.34, 23.0],
                                [-7.59, 3.39, 23.0],
                                [-7.57, 2.43, 23.0],
                                [-7.55, 1.47, 23.0],
                                [-7.54, 0.51, 23.0],
                                [-7.52, -0.45, 23.0],
                                [-7.54, -1.4, 23.0],
                                [-7.57, -2.35, 23.0],
                                [-7.59, -3.3, 23.0],
                                [-7.62, -4.25, 23.0],
                                [-6.61, -4.17, 23.0],
                                [-5.6, -4.09, 23.0],
                                [-4.59, -4.02, 23.0],
                                [-3.58, -3.94, 23.0],
                                [-2.65, -3.91, 23.0],
                                [-1.72, -3.88, 23.0],
                                [-0.78, -3.84, 23.0],
                                [0.15, -3.81, 23.0],
                                [1.11, -3.83, 23.0],
                                [2.07, -3.84, 23.0],
                                [3.03, -3.86, 23.0],
                                [3.99, -3.88, 23.0],
                                [3.97, -2.97, 23.0],
                                [3.96, -2.06, 23.0],
                                [3.94, -1.15, 23.0],
                                [3.92, -0.24, 23.0],
                                [3.91, 0.76, 23.0],
                                [3.91, 1.77, 23.0],
                                [3.9, 2.77, 23.0],
                                [3.89, 3.78, 23.0],
                                [2.94, 3.72, 23.0],
                                [1.99, 3.66, 23.0],
                                [1.04, 3.6, 23.0],
                                [0.09, 3.54, 23.0],
                                [-0.85, 3.49, 23.0],
                                [-1.78, 3.44, 23.0],
                                [-2.72, 3.39, 23.0],
                                [-3.66, 3.34, 23.0],
                                [-3.66, 2.39, 23.0],
                                [-3.67, 1.44, 23.0],
                                [-3.67, 0.48, 23.0],
                                [-3.67, -0.47, 23.0],
                                [-2.71, -0.43, 23.0],
                                [-1.75, -0.39, 23.0],
                                [-0.8, -0.35, 23.0],
                                [0.16, -0.31, 23.0]
                                ]
                self.setpoint = self.waypoint[self.current_waypoint_indx]   #This is the list used for storing the setpoint to which the drone is supposed to arrive at.

                
                #This FLAG variable is used in a way so that the setpoint is only set once and not set multiple times and hence preventing from moving to the next point
                self.waypoint_decided=True

        if 0<self.takeoff_x and not self.waypoint_decided:
            if 0<self.takeoff_y:
                self.waypoint = [
                                [7.68, 7.63, 23.0],
                                [6.74, 7.62, 23.0],
                                [5.8, 7.61, 23.0],
                                [4.86, 7.59, 23.0],
                                [3.92, 7.58, 23.0],
                                [2.93, 7.55, 23.0],
                                [1.94, 7.53, 23.0],
                                [0.94, 7.5, 23.0],
                                [-0.05, 7.47, 23.0],
                                [-0.99, 7.39, 23.0],
                                [-1.92, 7.32, 23.0],
                                [-2.85, 7.25, 23.0],
                                [-3.79, 7.17, 23.0],
                                [-4.78, 7.17, 23.0],
                                [-5.77, 7.17, 23.0],
                                [-6.76, 7.18, 23.0],
                                [-7.75, 7.18, 23.0],
                                [-7.71, 6.23, 23.0],
                                [-7.67, 5.29, 23.0],
                                [-7.63, 4.34, 23.0],
                                [-7.59, 3.39, 23.0],
                                [-7.57, 2.43, 23.0],
                                [-7.55, 1.47, 23.0],
                                [-7.54, 0.51, 23.0],
                                [-7.52, -0.45, 23.0],
                                [-7.54, -1.4, 23.0],
                                [-7.57, -2.35, 23.0],
                                [-7.59, -3.3, 23.0],
                                [-7.62, -4.25, 23.0],
                                [-7.6, -5.24, 23.0],
                                [-7.58, -6.23, 23.0],
                                [-7.55, -7.22, 23.0],
                                [-7.53, -8.21, 23.0],
                                [-7.49, -8.17, 23.0],
                                [-7.45, -8.12, 23.0],
                                [-7.41, -8.07, 23.0],
                                [-7.37, -8.03, 23.0],
                                [-5.46, -8.0, 23.0],
                                [-3.54, -7.98, 23.0],
                                [-1.63, -7.96, 23.0],
                                [0.28, -7.93, 23.0],
                                [1.23, -7.88, 23.0],
                                [2.19, -7.83, 23.0],
                                [3.14, -7.78, 23.0],
                                [4.1, -7.73, 23.0],
                                [5.07, -7.71, 23.0],
                                [6.04, -7.7, 23.0],
                                [7.0, -7.68, 23.0],
                                [7.97, -7.66, 23.0],
                                [7.92, -6.71, 23.0],
                                [7.88, -5.77, 23.0],
                                [7.83, -4.82, 23.0],
                                [7.78, -3.87, 23.0],
                                [7.77, -2.94, 23.0],
                                [7.76, -2.0, 23.0],
                                [7.75, -1.06, 23.0],
                                [7.74, -0.13, 23.0],
                                [7.71, 0.79, 23.0],
                                [7.68, 1.71, 23.0],
                                [7.65, 2.62, 23.0],
                                [7.62, 3.54, 23.0],
                                [6.69, 3.6, 23.0],
                                [5.75, 3.66, 23.0],
                                [4.82, 3.72, 23.0],
                                [3.89, 3.78, 23.0],
                                [2.94, 3.72, 23.0],
                                [1.99, 3.66, 23.0],
                                [1.04, 3.6, 23.0],
                                [0.09, 3.54, 23.0],
                                [-0.85, 3.49, 23.0],
                                [-1.78, 3.44, 23.0],
                                [-2.72, 3.39, 23.0],
                                [-3.66, 3.34, 23.0],
                                [-3.66, 2.39, 23.0],
                                [-3.67, 1.44, 23.0],
                                [-3.67, 0.48, 23.0],
                                [-3.67, -0.47, 23.0],
                                [-3.65, -1.34, 23.0],
                                [-3.62, -2.21, 23.0],
                                [-3.6, -3.07, 23.0],
                                [-3.58, -3.94, 23.0],
                                [-2.65, -3.91, 23.0],
                                [-1.72, -3.88, 23.0],
                                [-0.78, -3.84, 23.0],
                                [0.15, -3.81, 23.0],
                                [1.11, -3.83, 23.0],
                                [2.07, -3.84, 23.0],
                                [3.03, -3.86, 23.0],
                                [3.99, -3.88, 23.0],
                                [3.97, -2.97, 23.0],
                                [3.96, -2.06, 23.0],
                                [3.94, -1.15, 23.0],
                                [3.92, -0.24, 23.0],
                                [2.98, -0.26, 23.0],
                                [2.04, -0.28, 23.0],
                                [1.1, -0.29, 23.0],
                                [0.16, -0.31, 23.0]
                                ]
                self.setpoint = self.waypoint[self.current_waypoint_indx]   #This is the list used for storing the setpoint to which the drone is supposed to arrive at.

                self.waypoint_decided=True

        if 0<self.takeoff_x and not self.waypoint_decided:
            if 0>self.takeoff_y:
                self.waypoint = [
                                [7.97, -7.66, 23.0],
                                [7.92, -6.71, 23.0],
                                [7.88, -5.77, 23.0],
                                [7.83, -4.82, 23.0],
                                [7.78, -3.87, 23.0],
                                [7.77, -2.94, 23.0],
                                [7.76, -2.0, 23.0],
                                [7.75, -1.06, 23.0],
                                [7.74, -0.13, 23.0],
                                [7.71, 0.79, 23.0],
                                [7.68, 1.71, 23.0],
                                [7.65, 2.62, 23.0],
                                [7.62, 3.54, 23.0],
                                [7.63, 4.56, 23.0],
                                [7.65, 5.58, 23.0],
                                [7.67, 6.61, 23.0],
                                [7.68, 7.63, 23.0],
                                [6.74, 7.62, 23.0],
                                [5.8, 7.61, 23.0],
                                [4.86, 7.59, 23.0],
                                [3.92, 7.58, 23.0],
                                [2.93, 7.55, 23.0],
                                [1.94, 7.53, 23.0],
                                [0.94, 7.5, 23.0],
                                [-0.05, 7.47, 23.0],
                                [-0.99, 7.39, 23.0],
                                [-1.92, 7.32, 23.0],
                                [-2.85, 7.25, 23.0],
                                [-3.79, 7.17, 23.0],
                                [-4.78, 7.17, 23.0],
                                [-5.77, 7.17, 23.0],
                                [-6.76, 7.18, 23.0],
                                [-7.75, 7.18, 23.0],
                                [-7.71, 6.23, 23.0],
                                [-7.67, 5.29, 23.0],
                                [-7.63, 4.34, 23.0],
                                [-7.59, 3.39, 23.0],
                                [-7.57, 2.43, 23.0],
                                [-7.55, 1.47, 23.0],
                                [-7.54, 0.51, 23.0],
                                [-7.52, -0.45, 23.0],
                                [-7.54, -1.4, 23.0],
                                [-7.57, -2.35, 23.0],
                                [-7.59, -3.3, 23.0],
                                [-7.62, -4.25, 23.0],
                                [-7.6, -5.24, 23.0],
                                [-7.58, -6.23, 23.0],
                                [-7.55, -7.22, 23.0],
                                [-7.53, -8.21, 23.0],
                                [-7.49, -8.17, 23.0],
                                [-7.45, -8.12, 23.0],
                                [-7.41, -8.07, 23.0],
                                [-7.37, -8.03, 23.0],
                                [-5.46, -8.0, 23.0],
                                [-3.54, -7.98, 23.0],
                                [-1.63, -7.96, 23.0],
                                [0.28, -7.93, 23.0],
                                [1.23, -7.88, 23.0],
                                [2.19, -7.83, 23.0],
                                [3.14, -7.78, 23.0],
                                [4.1, -7.73, 23.0],
                                [4.07, -6.77, 23.0],
                                [4.04, -5.8, 23.0],
                                [4.02, -4.84, 23.0],
                                [3.99, -3.88, 23.0],
                                [3.97, -2.97, 23.0],
                                [3.96, -2.06, 23.0],
                                [3.94, -1.15, 23.0],
                                [3.92, -0.24, 23.0],
                                [3.91, 0.76, 23.0],
                                [3.91, 1.77, 23.0],
                                [3.9, 2.77, 23.0],
                                [3.89, 3.78, 23.0],
                                [2.94, 3.72, 23.0],
                                [1.99, 3.66, 23.0],
                                [1.04, 3.6, 23.0],
                                [0.09, 3.54, 23.0],
                                [-0.85, 3.49, 23.0],
                                [-1.78, 3.44, 23.0],
                                [-2.72, 3.39, 23.0],
                                [-3.66, 3.34, 23.0],
                                [-3.66, 2.39, 23.0],
                                [-3.67, 1.44, 23.0],
                                [-3.67, 0.48, 23.0],
                                [-3.67, -0.47, 23.0],
                                [-1.75, -1.32, 23.0],
                                [0.16, -2.17, 23.0],
                                [2.08, -3.03, 23.0],
                                [3.99, -3.88, 23.0],
                                [3.03, -3.86, 23.0],
                                [2.07, -3.84, 23.0],
                                [1.11, -3.83, 23.0],
                                [0.15, -3.81, 23.0],
                                [0.15, -2.94, 23.0],
                                [0.15, -2.06, 23.0],
                                [0.16, -1.19, 23.0],
                                [0.16, -0.31, 23.0]
                                ]
                
                self.setpoint = self.waypoint[self.current_waypoint_indx]   #This is the list used for storing the setpoint to which the drone is supposed to arrive at.

                self.waypoint_decided=True
               
        if 0>self.takeoff_x and not self.waypoint_decided:
            if 0<self.takeoff_y:
                self.waypoint = [
                                [-7.75, 7.18, 23.0],
                                [-7.71, 6.23, 23.0],
                                [-7.67, 5.29, 23.0],
                                [-7.63, 4.34, 23.0],
                                [-7.59, 3.39, 23.0],
                                [-7.57, 2.43, 23.0],
                                [-7.55, 1.47, 23.0],
                                [-7.54, 0.51, 23.0],
                                [-7.52, -0.45, 23.0],
                                [-7.54, -1.4, 23.0],
                                [-7.57, -2.35, 23.0],
                                [-7.59, -3.3, 23.0],
                                [-7.62, -4.25, 23.0],
                                [-7.6, -5.24, 23.0],
                                [-7.58, -6.23, 23.0],
                                [-7.55, -7.22, 23.0],
                                [-7.53, -8.21, 23.0],
                                [-7.49, -8.17, 23.0],
                                [-7.45, -8.12, 23.0],
                                [-7.41, -8.07, 23.0],
                                [-7.37, -8.03, 23.0],
                                [-5.46, -8.0, 23.0],
                                [-3.54, -7.98, 23.0],
                                [-1.63, -7.96, 23.0],
                                [0.28, -7.93, 23.0],
                                [1.23, -7.88, 23.0],
                                [2.19, -7.83, 23.0],
                                [3.14, -7.78, 23.0],
                                [4.1, -7.73, 23.0],
                                [5.07, -7.71, 23.0],
                                [6.04, -7.7, 23.0],
                                [7.0, -7.68, 23.0],
                                [7.97, -7.66, 23.0],
                                [7.92, -6.71, 23.0],
                                [7.88, -5.77, 23.0],
                                [7.83, -4.82, 23.0],
                                [7.78, -3.87, 23.0],
                                [7.77, -2.94, 23.0],
                                [7.76, -2.0, 23.0],
                                [7.75, -1.06, 23.0],
                                [7.74, -0.13, 23.0],
                                [7.71, 0.79, 23.0],
                                [7.68, 1.71, 23.0],
                                [7.65, 2.62, 23.0],
                                [7.62, 3.54, 23.0],
                                [7.63, 4.56, 23.0],
                                [7.65, 5.58, 23.0],
                                [7.67, 6.61, 23.0],
                                [7.68, 7.63, 23.0],
                                [6.74, 7.62, 23.0],
                                [5.8, 7.61, 23.0],
                                [4.86, 7.59, 23.0],
                                [3.92, 7.58, 23.0],
                                [2.93, 7.55, 23.0],
                                [1.94, 7.53, 23.0],
                                [0.94, 7.5, 23.0],
                                [-0.05, 7.47, 23.0],
                                [-0.99, 7.39, 23.0],
                                [-1.92, 7.32, 23.0],
                                [-2.85, 7.25, 23.0],
                                [-3.79, 7.17, 23.0],
                                [-3.76, 6.21, 23.0],
                                [-3.73, 5.25, 23.0],
                                [-3.69, 4.3, 23.0],
                                [-3.66, 3.34, 23.0],
                                [-3.66, 2.39, 23.0],
                                [-3.67, 1.44, 23.0],
                                [-3.67, 0.48, 23.0],
                                [-3.67, -0.47, 23.0],
                                [-3.65, -1.34, 23.0],
                                [-3.62, -2.21, 23.0],
                                [-3.6, -3.07, 23.0],
                                [-3.58, -3.94, 23.0],
                                [-2.65, -3.91, 23.0],
                                [-1.72, -3.88, 23.0],
                                [-0.78, -3.84, 23.0],
                                [0.15, -3.81, 23.0],
                                [1.11, -3.83, 23.0],
                                [2.07, -3.84, 23.0],
                                [3.03, -3.86, 23.0],
                                [3.99, -3.88, 23.0],
                                [3.97, -2.97, 23.0],
                                [3.96, -2.06, 23.0],
                                [3.94, -1.15, 23.0],
                                [3.92, -0.24, 23.0],
                                [3.91, 0.76, 23.0],
                                [3.91, 1.77, 23.0],
                                [3.9, 2.77, 23.0],
                                [3.89, 3.78, 23.0],
                                [2.94, 3.72, 23.0],
                                [1.99, 3.66, 23.0],
                                [1.04, 3.6, 23.0],
                                [0.09, 3.54, 23.0],
                                [0.11, 2.58, 23.0],
                                [0.12, 1.61, 23.0],
                                [0.14, 0.65, 23.0],
                                [0.16, -0.31, 23.0]
                                ]
                
                self.setpoint = self.waypoint[self.current_waypoint_indx]   #This is the list used for storing the setpoint to which the drone is supposed to arrive at.

                self.waypoint_decided=True

        #The following "if" statements are used to blink the LED and beep the buzzer according the number of LED detected by the drone
        if self.a_detected:     #If this FLAG variable is set to TRUE it means that alien_a is detected and hence only two LED and two buzzer beeps are made

            #The self.aligned_time variable is used to store the time at which the drone had aligned with an LED
            #Keeping aligned time as a reference we check how much time has passed and accordingly toggle the values of AUX3 and AUX4 which results in beep and blink
            if time.time()-self.aligned_time>=0.1:   
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.2:   
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.3:
                self.rc_message.aux3 = 1000
        if self.a_detected:
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=0.6:
                self.rc_message.aux4 = 1000

            #Keeping aligned time as a reference we check how much time has passed and accordingly toggle the values of AUX3 and AUX4 which results in beep and blink
        if self.b_detected:     #If this FLAG variable is set to TRUE it means that alien_b is detected and hence only three LED and two buzzer beeps are made
            if time.time()-self.aligned_time>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.3:
                self.rc_message.aux3 = 1000
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.5:
                self.rc_message.aux3 = 1000
        if self.b_detected:
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=0.6:
                self.rc_message.aux4 = 1000
            if time.time()-self.aligned_time>=0.8:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=1:
                self.rc_message.aux4 = 1000

            #Keeping aligned time as a reference we check how much time has passed and accordingly toggle the values of AUX3 and AUX4 which results in beep and blink
        if self.c_detected:     #If this FLAG variable is set to TRUE it means that alien_c is detected and hence only four LED and two buzzer beeps are made
            if time.time()-self.aligned_time>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.3:
                self.rc_message.aux3 = 1000
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.5:
                self.rc_message.aux3 = 1000
            if time.time()-self.aligned_time>=0.6:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.7:
                self.rc_message.aux3 = 1000
        if self.c_detected:
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=0.6:
                self.rc_message.aux4 = 1000
            if time.time()-self.aligned_time>=0.8:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=1:
                self.rc_message.aux4 = 1000
            if time.time()-self.aligned_time>=1.2:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=1.4:
                self.rc_message.aux4 = 1000

            #Keeping aligned time as a reference we check how much time has passed and accordingly toggle the values of AUX3 and AUX4 which results in beep and blink
        if self.d_detected:     ##If this FLAG variable is set to TRUE it means that alien_d is detected and hence only five LED and two buzzer beeps are made
            if time.time()-self.aligned_time>=0.1:
                self.rc_message.aux3 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.3:
                self.rc_message.aux3 = 1000
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.6:
                self.rc_message.aux3 = 1000
            if time.time()-self.aligned_time>=0.7:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=0.8:
                self.rc_message.aux3 = 1000
            if time.time()-self.aligned_time>=0.9:
                self.rc_message.aux3 = 2000
            if time.time()-self.aligned_time>=1:
                self.rc_message.aux3 = 1000
        if self.d_detected:
            if time.time()-self.aligned_time>=0.2:
                self.rc_message.aux4 = 1000
                self.cu_time=time.time()
            if time.time()-self.aligned_time>=0.4:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=0.6:
                self.rc_message.aux4 = 1000
            if time.time()-self.aligned_time>=0.8:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=1:
                self.rc_message.aux4 = 1000
            if time.time()-self.aligned_time>=1.2:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=1.4:
                self.rc_message.aux4 = 1000
            if time.time()-self.aligned_time>=1.6:
                self.rc_message.aux4 = 1500
            if time.time()-self.aligned_time>=1.8:
                self.rc_message.aux4 = 1000

        print("INSIDE PID LOOOOP")
        print(self.detected_waypoint_index)
        print(self.current_waypoint_indx)
        print(self.camera_x,self.camera_y)
        print(self.avoid_aligning)
    
        #This part of the code is responsible for the increment of the setpoint from the waypoint and hence enabling the search algorithm
        if self.waypoint_decided:
            if self.current_waypoint_indx < len(self.waypoint):     #Check if the end of waypoint is reached
                if self.at_setpoint() and not self.in_hover_mode:   #Check if the drone is already in hovering mode which occurs when a setpoint is reached and the drone hovers there for a given duration
                    self.hover_start_time = time.time()
                    self.in_hover_mode = True
                elif self.in_hover_mode and (time.time() - self.hover_start_time) >= self.hover_duration:   #This line checks if the drone has hovered for the given time and move on to next setpoint if hovering has finished
                    self.in_hover_mode = False
                    self.current_waypoint_indx += 1     #This line ensures that if the drone has reached it's current setpoint and hovered for a particular period of time then the drone moves on to the next setpoint
                    if self.current_waypoint_indx < len(self.waypoint):  
                        self.setpoint = self.waypoint[self.current_waypoint_indx]   #This line sets the new setpoint


        #This part of the code checks if all the aliens are detected or not by checking the length of the list_alien and checks if the drone is actually inside the search algorithm and not in the landing algorithm
        #If the conditions are true then the drone starts the wayppoint for landing
        if len(self.list_alien)==5 and not self.started_landing:
            self.landing_prep()


        #This part of the code ensure that after reaching the final landing setpoint the drone starts the safe landing process
        if self.setpoint == [7,9,25.5]:
            if self.at_setpoint():
                print("LANDING FOR FINAL SETPOINT")
                self.start_landing()
                        
                
        try:

            #This part of the code checks if the drone has moved away from the already aligned LED and only then it allows the drone to align to the next LED
            if self.current_waypoint_indx >= self.detected_waypoint_index+6:        #Here the code checks if the drone has moved atleast 6 waypoints ahead before enabling the detection of the next LED
                print("ALIEN_BLOCK")
                self.avoid_aligning=False

            #The below statements are used to calculate the error in coordinates of the drone with respect to that of the setpoint
            self.error[0] = -(self.drone_whycon_pose_array.poses[0].position.x - self.setpoint[0] )
            self.error[1] = (self.drone_whycon_pose_array.poses[0].position.y - self.setpoint[1] )
            self.error[2] = -(self.setpoint[2] - self.drone_whycon_pose_array.poses[0].position.z) 
        except:
            pass
        print(self.rc_message.rc_throttle)


                #CALCULATION OF PROPORTIONAL TERM FOR PID CONTROL
        

        self.proportional[0] = (self.Kp[0] * self.error[0])
        self.proportional[1] = (self.Kp[1] * self.error[1])
        self.proportional[2] = (self.Kp[2] * self.error[2])


                #CALCULATION OF INTEGRAL TERM FOR PID CONTROL
        

        self.integral[0] = self.sum_error[0]*self.Ki[0]
        self.integral[1] = self.sum_error[1]*self.Ki[1]
        self.integral[2] = self.sum_error[2]*self.Ki[2]


                #CALCULATION OF DERRIVATIVE TERM FOR PID CONTROL
        

        self.derrivative[0] = (self.prev_error[0] - self.error[0])*self.Kd[0]
        self.derrivative[1] = (self.prev_error[1] - self.error[1])*self.Kd[1]
        self.derrivative[2] = ((self.prev_error[2] - self.error[2]))*self.Kd[2]


        #This part adds up the errors in all three axes and calculates the sum for using in the integral term
        self.sum_error[0]=self.sum_error[0] + self.error[0]
        self.sum_error[1]=self.sum_error[1] + self.error[1]
        self.sum_error[2]=self.sum_error[2] + self.error[2]


            #ANTI-WIND UP METHOD

        #This part of the code uses pre defined LIMITS for the integral part and when the integral part exceeds the limit the maximum value is set
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

        #This part of the code calculates the values to be published according to the Base value, proportional, integral, derrivative terms and store it in a variable
        self.rc_throttle = 1450 +int(self.proportional[2] + self.integral[2] - self.derrivative[2])
        self.rc_pitch = 1492.5 +int(self.proportional[1] + self.integral[1] - self.derrivative[1])
        self.rc_roll = 1502.5 +int(self.proportional[0] + self.integral[0] - self.derrivative[0])

#------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
        #Here the throttle, roll and pitch values are copied into a local variable which is used later as inputs to the butterworth filter
        throttle = self.rc_throttle
        roll = self.rc_roll
        pitch = self.rc_pitch

        #This line copies the the current error and saves it in the prev_error list which is then used for calculation of the derrivative term
        self.prev_error = copy.deepcopy(self.error)

        #The YAW value is made constant 1500 to avoid any yaw movement as it is not required for this Task
        self.rc_message.rc_yaw = int(1500)


        #The below part of the code ensures that the Butterworth filter gets it's inputs under a regulated state because not doing so adds the rc commands up to a huge value inside the filter,
        # causing the drone to switch off as it starts exceeding the maximum integer value limit.
        if roll > MAX_ROLL_FILTER:     
            roll = MAX_ROLL_FILTER
        elif roll < MIN_ROLL_FILTER:
            roll = MIN_ROLL_FILTER
        
        if pitch > MAX_PITCH_FILTER:     
            pitch = MAX_PITCH_FILTER
        elif pitch < MIN_PITCH_FILTER:
            pitch = MIN_PITCH_FILTER

        if throttle > MAX_THROTTLE_FILTER:     
            throttle = MAX_THROTTLE_FILTER
        elif throttle < MIN_THROTTLE_FILTER:
            throttle = MIN_THROTTLE_FILTER


                                 # BUTTERWORTH FILTER
        
        span = 15       #This variable is used to set the Number of values the filter considers at a time 

        for index, val in enumerate([roll, pitch, throttle]):
            DRONE_WHYCON_POSE[index].append(val)
            if len(DRONE_WHYCON_POSE[index]) == span:
                DRONE_WHYCON_POSE[index].pop(0)
            if len(DRONE_WHYCON_POSE[index]) != span-1:
                return
            
            #This part sets the intrinsic parameters of the Butterworth filter
            order = 3       #Order of the filter
            fs = 30         #Sampling frequency
            fc = 5          #Cut-Off frequency
            nyq = 0.5 * fs  #Applying Nyquist Criterion
            wc = fc / nyq   #Angular cut-off frequency
            b, a = scipy.signal.butter(N=order, Wn=wc, btype='lowpass', analog=False, output='ba')
            filtered_signal = scipy.signal.lfilter(b, a, DRONE_WHYCON_POSE[index])

            #Assigning the filtered outputs to the rc commands
            if index == 0:
                self.rc_message.rc_roll = int(filtered_signal[-1])

            elif index == 1:
                self.rc_message.rc_pitch = int(filtered_signal[-1])

            if index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])

        #This Limits the output rc commands to the drone so that the motors doesnot heat much.
        if self.rc_message.rc_roll > MAX_ROLL:     
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL
        
        if self.rc_message.rc_pitch > MAX_PITCH:     
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH

        if self.rc_message.rc_throttle > MAX_THROTTLE:    
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE

        #This assigns an integer value to the output message because only integer values are supported by this type of message
        self.rc_msg.rc_roll = int(roll)
        self.rc_msg.rc_pitch = int(pitch)
        self.rc_msg.rc_throttle = int(throttle)

        #The errors in all the three axes are published in PIDError for fine tuning and other purposes in plotjuggler
        self.pid_error_pub.publish(
            PIDError(
                roll_error=float(self.error[0]),
                pitch_error=float(self.error[1]),
                throttle_error=float(self.error[2]),
                yaw_error=-0.5,
                zero_error=0.0,
            )
        )

        #Publishing the RC commands to the drone for the PID control 
        self.rc_pub.publish(self.rc_message)
        self.rc_unfilter.publish(self.rc_msg)

        print(self.hover_duration)
        print(self.setpoint)   
        print(self.list_alien)     

    
    ''' 
        * Function Name: shutdown_hook
        * Input: None
        * Output: None
        * Logic: This function is used to call the disarm function.
        * Example Call: self.shutdown_hook()
    '''
    def shutdown_hook(self):
        #This block of code is present for printing the information that the code is entering the disarming loop in the terminal.
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

    ''' 
        * Function Name: disarm
        * Input: None
        * Output: None
        * Logic: This function is used to disarm the drone .
        * Example Call: self.disarm()
    '''
    def disarm(self):
        # Function to disarm the drone 
        # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
        # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and Actuators stop 
        self.rc_message.rc_roll=MIN_ROLL
        self.rc_message.rc_pitch=MIN_PITCH
        self.rc_message.rc_throttle=MIN_THROTTLE
        self.node.get_logger().info("Calling DISARM service")
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)
    
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
    desired_rate = 50.0     #Specify the rate at which the code must execute
    timer_period = rclpy.duration.Duration(seconds=1.0/desired_rate)
    timer = node.create_timer(timer_period.nanoseconds*1e-9, controller.pid)
    try:
        while rclpy.ok():
                if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 10:
                    node.get_logger().error("Unable to detect WHYCON poses")
                rclpy.spin_once(node) 
    except Exception as err:
        print(err)
    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()