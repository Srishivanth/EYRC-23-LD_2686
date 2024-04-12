#!/usr/bin/env python3

"""
Controller for the drone
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

DERRIVATIVE_LIMIT = 150


DRONE_WHYCON_POSE = [[], [], []]

# Similarly, create upper and lower limits, base value, and max sum error values for roll and pitch

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
               # Setpoints for x, y, z respectively      
        qos_profile = QoSProfile(depth=10)  # Adjust depth as needed

        self.error = [0.0, 0.0, 0.0]         # Error for roll, pitch and throttle        

        # Create variables for integral and differential error

        self.integral = [0.0,0.0,0.0]
        self.proportional = [0.0,0.0,0.0]
        self.derrivative = [0.0,0.0,0.0]
        self.prev_error = [0.0,0.0,0.0]
        self.last_time = 0
        self.timer=True
        
        self.final_throttle=0
        self.final_roll=0
        self.final_pitch=0

        self.lock=True
        self.new_lock = True
        self.sum_error=[0,0,0]
        self.drone_position =[0,0,0]


        self.rc_message.aux1 = 1500
        self.rc_message.aux2 = 1500
        self.rc_message.aux3 = 1000
        self.rc_message.aux4 = 2000

        self.hover_duration = 5
        self.in_hover_mode = False

        self.alien_type=''
        self.biolocation_msg=Biolocation()
        self.biolocation_msg.organism_type=''
        self.biolocation_msg.whycon_x=0.0
        self.biolocation_msg.whycon_y=0.0
        self.biolocation_msg.whycon_z=0.0

        self.maxx_len=0


        #self.fx=1233.2887188967234
        #self.fy=1233.2887188967234
        self.cx=252.75
        self.cy=262.19

        self.fx=1459.52
        self.fy=1460.72
        #self.cx=250.5
        #self.cy=250.5

        #self.fx = 2448.5
        #self.fy = 2448.5
        #self.fx = 11.48
        #self.fy = 11.48
        #self.cx = 1640
        #self.cy = 1232

        #self.fx=3.04
        #self.fy=3.04
        #self.cx=3280/2
        #self.cy=2464/2

        #self.fx=990.9387
        #self.fy=1042.6280
        #self.cx=980.9874
        #self.cy=522.4008
        self.camera_x=0.1
        self.camera_y=0.1
        self.error_x=0
        self.error_y=0

        self.len_cnts=0
        self.max_len=0
        self.clear_block=False

        self.led_detected = False

        self.new__lock=True

        self.new_new_lock = True

        self.buzz_lock = True

        self.align_throttle =0

        self.list_alien=[0.5]

        self.new_lock_lock = True

        qos_profile = QoSProfile(depth=10)  # Adjust depth as needed



        self.waypoint = [[0,0,20],[-1.5,0,20],
                         
            [-3,0,20],
            
            [-3,-2.66,20],
            [0,-2.66,20],
            [3,-2.66,20],
            [3,0,20],
            [3,2.66,20],
            [0,2.66,20],
            [-3,2.66,20],

            [-6,2.66,20],
            [-6,0,20],
            [-6,-2.66,20],
            [-6,-5.33,20],
            [-3,-5.33,20],
            [0,-5.33,20],
            [3,-5.33,20],
            [6,-5.33,20],
            [6,-2.66,20],
            [6,0,20],
            [6,2.66,20],
            [6,5.33,20],
            [3,5.33,20],
            [0,5.33,20],
            [-3,5.33,20],
            [-6,5.33,20],
            [-9,5.33,20],
            [-9,2.66,20],
            [-9,0,20],
            [-9,-2.66,20],
                         
            
            
            [-9,-5.33,20],
            [-9,-8,20],
            [-6,-8,20],
            [-3,-8,20],
            [0,-8,20],
            [3,-8,20],
            [6,-8,20],
            [9,-8,20],
            [9,-5.33,20],
            [9,-2.66,20],
            [9,0,20],
            [9,2.66,20],
            [9,5.33,20],
            [9,8,20],
            [6,8,20],
            [3,8,20],
            [0,8,20],
            [-3,8,20],
            [-6,8,20],
            [-9,8,20],

            [-9,5.33,20],
            [-9,2.66,20],
            [-9,0,20],
            [-9,-2.66,20],
            [-9,-5.33,20],
            [-9,-8,20],
            [-6,-8,20],
            [-3,-8,20],
            [0,-8,20],

            

            [3,-8,20],
            [6,-8,20],
            [9,-8,20],
            [9,-5.33,20],
            [9,-2.66,20],
            [9,0,20],
            [9,2.66,20],
            [9,5.33,20],
            [9,8,20],
            [6,8,20],
            [3,8,20],
            [0,8,20],
            [-3,8,20],
            [-6,8,20],
            [-9,8,20]]
            #[0,-2.25,26],
            #[2.25,-2.25,26],
            #[2.25,0.0,26],
            #[2.25,2.25,26],
            #[0,2.25,26],
            #[-2.25,2.25,26],
            #[-2.25,0,26],
            #[-2.25,-2.25,26]]
        
        self.current_waypoint_indx=0
        self.setpoint = self.waypoint[self.current_waypoint_indx]

        # Create variables for previous error and sum_error

        #self.Kp = [ 350 * 0.01  ,  350 * 0.01  ,  220*0.005   ]

        # Similarly create variables for Kd and Ki

        #self.Ki = [ 30 * 0.01  ,  30 * 0.01  ,  401 * 0.0000001  ]
        #self.Kd = [ 722 * 0.25  ,  722 * 0.25  ,  1200 * 0.35  ]


        self.Kp = [9.06,4.16,8.8] #[406,416,17.6]
        self.Kd = [467.1,386.5,448.5] #[4171,3865,4485]
        self.Ki = [0.075,0.0066,0.0298] #[8,66,298]

        #self.Kp = [0,4.16,8.8] #[406,416,17.6]
        #self.Kd = [0,386.5,448.5] #[4171,3865,4485]
        #self.Ki = [0,0.0066,0.0298] #[8,66,298]

 

        #self.Kp = [0,0,0]
        #self.Kd = [0,0,0]
        #self.Ki = [0,0,0]

        #self.Kp = [ 195 * 0.03  ,  249 * 0.01  ,  280 * 0.01 ]
#
        ## Similarly create variables for Kd and Ki
#
        #self.Ki = [ 63 * 0.0005  ,  39 * 0.0  ,  401 * 0.0001  ]
        #self.Kd = [ 720 * 0.1  ,  722 * 0.1  ,  1681 * 0.1  ]

        self.Base_z=0
        self.earliest_z = None
        self.hover_start = 0
        #if self.whycon_poses_callback.poses[0]:
#
        #else:
        #    pass


        # Create subscriber for WhyCon 
        
        self.whycon_sub = node.create_subscription(PoseArray,"/whycon/poses",self.whycon_poses_callback,1)
        
        # Similarly create subscribers for pid_tuning_altitude, pid_tuning_roll, pid_tuning_pitch and any other subscriber if required
       
        self.pid_alt = node.create_subscription(PidTune,"/pid_tuning_altitude",self.pid_tune_throttle_callback,1)
        self.pid_roll= node.create_subscription(PidTune,"/pid_tuning_roll",self.pid_tune_roll_callback,1)
        self.pid_pitch = node.create_subscription(PidTune,"/pid_tuning_pitch",self.pid_tune_pitch_callback,1)
        self.biolocation_publisher = node.create_publisher(Biolocation,'/astrobiolocation',10)

        # Create publisher for sending commands to drone 

        self.rc_pub = node.create_publisher(RCMessage, "/swift/rc_command",1)
        self.rc_unfilter = node.create_publisher(RCMessage, "/swift/rc_unfilter",1)

        # Create publisher for publishing errors for plotting in plotjuggler 
        
        self.pid_error_pub = node.create_publisher(PIDError, "/luminosity_drone/pid_error",1)    

        self.limit = node.create_publisher(PIDError,"/limits",1)    

        self.image_sub =node.create_subscription(Image, '/video_frames', self.image_callback,10)


        #self.throttle_error_pub = node.create_publisher(FloatingPointError, "/alt_error", 1)
        #self.roll_error_pub = node.create_publisher(FloatingPointError, "/roll_error", 1)
        #self.pitch_error_pub = node.create_publisher(FloatingPointError, "/pitch_error", 1)
#
    #def set_base_z(self):
    #    if self.drone_whycon_pose_array.poses:
    #        self.Base_z = self.drone_whycon_pose_array.poses[0].position.z
    #        self.node.get_logger().info("Base station coordinate set to:", self.Base_z)
    #    else:
    #        self.node.get_logger().info("Waiting for WhyCon pose data...")
     


    def whycon_poses_callback(self, msg):
        self.last_whycon_pose_received_at = self.node.get_clock().now().seconds_nanoseconds()[0]
        self.drone_whycon_pose_array = msg
        self.drone_position[0] = self.drone_whycon_pose_array.poses[0].position.x
        self.drone_position[1] = self.drone_whycon_pose_array.poses[0].position.y
        self.drone_position[2] = self.drone_whycon_pose_array.poses[0].position.z
        self.whycon_data_recieved = True


    def pid_tune_throttle_callback(self, msg):
        self.Kp[2] = msg.kp * 0.05
        self.Ki[2] = msg.ki * 0.0001
        self.Kd[2] = msg.kd * 0.5

    # Similarly add callbacks for other subscribers
        
    def pid_tune_roll_callback(self, msg):
        self.Kp[0] = msg.kp * 0.01
        self.Ki[0] = msg.ki * 0.0001
        self.Kd[0] = msg.kd * 0.1

    def pid_tune_pitch_callback(self, msg):
        self.Kp[1] = msg.kp * 0.01
        self.Ki[1] = msg.ki * 0.0001
        self.Kd[1] = msg.kd * 0.1

    def at_setpoint(self):
        #print("checking for setpoint")
        x_tolerance = 0.6
        y_tolerance = 0.6
        z_tolerance = 0.6
        x_at_setpoint = abs((self.setpoint[0]) - self.drone_position[0]) <= x_tolerance 
        y_at_setpoint = abs((self.setpoint[1]) - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs((self.setpoint[2]) - self.drone_position[2]) <= z_tolerance        
        #print(x_at_setpoint and y_at_setpoint and z_at_setpoint)
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    
    def at_new_setpoint(self):
        #print("checking for new setpoint")
        x_tolerance = 0.7
        y_tolerance = 0.7
        z_tolerance = 0.7
        x_at_setpoint = abs((self.setpoint[0]) - self.drone_position[0]) <= x_tolerance 
        y_at_setpoint = abs((self.setpoint[1]) - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs((self.setpoint[2]) - self.drone_position[2]) <= z_tolerance        
        #print(x_at_setpoint and y_at_setpoint and z_at_setpoint)
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    
    def at_new_new_setpoint(self):
        #print("checking for new setpoint")
        x_tolerance = 0.7
        y_tolerance = 0.7
        z_tolerance = 0.7
        x_at_setpoint = abs((self.setpoint[0]) - self.drone_position[0]) <= x_tolerance 
        y_at_setpoint = abs((self.setpoint[1]) - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs((self.setpoint[2]) - self.drone_position[2]) <= z_tolerance        
        #print(x_at_setpoint and y_at_setpoint and z_at_setpoint)
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
        
    
    def nextt(self):
        self.node.get_logger().info("RETURNING TO BASE")
        
        #self.Base_z=10
        current_setpoint =25
        while current_setpoint <=27:
            self.setpoint=[8.5,8.5,current_setpoint]
            while not self.at_setpoint():
                self.pid()
            current_setpoint+=0.5
            #print(current_setpoint,"th setpoint")
        self.node.get_logger().info("LANDING COMPLETE")
        self.shutdown_hook()




        


    def landing(self):
        start_time = time.time()
        print("LANDING PART")
        
        
        if time.time() - start_time >2:
            #pass
            self.shutdown_hook()

   

    def image_callback(self, data):
        self.desired_camera_x=0
        self.desired_camera_y=0
        
        image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
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
            
        if len(cnts) in [2,3,4,5] and self.clear_block==False:
            if len(cnts) not in self.list_alien:
                
                    print('hi')
                    self.in_hover_mode = True
                    self.hover_start_time = time.time()
                    if time.time()-self.in_hover_mode >=3:
                        self.in_hover_mode=False
                        self.stop_pid=True
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
        self.error_x = self.desired_camera_x - self.camera_x
        self.error_y = self.desired_camera_y - self.camera_y


    def aligning(self):

        
            if abs(self.camera_x)>0.00199999 and abs(self.camera_y)>0.001999999:
                print("aligning part")
                print(self.camera_x,self.camera_y)
                self.setpoint[0]=self.drone_position[0]+self.camera_x
                self.setpoint[1]=self.drone_position[1]+self.camera_y
                
            else:
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
                self.biolocation_msg.whycon_x = float(self.drone_position[0]+self.camera_x)
                self.biolocation_msg.whycon_y = float(self.drone_position[1]+self.camera_y)
                self.biolocation_msg.whycon_z = self.Base_z
                self.biolocation_publisher.publish(self.biolocation_msg)

                if self.alien_type == 'alien_a' and self.buzz_lock:

                    self.current_tim = time.time()
                    


                    i = 0  # Initialize the counter
                    start_time = time.time()  # Record the start time

                    while i < 2:  # Run the loop 4 times
                        current_time = time.time()  # Get the current time

                        if current_time - start_time >= 0.2:
                            self.rc_message.aux3 = 2000
                            self.rc_pub.publish(self.rc_message)


                            start_time = time.time()

                            while time.time() - start_time < 0.2:
                                pass
                            
                            self.rc_message.aux3 = 1000
                            self.rc_pub.publish(self.rc_message)


                            i += 1

                    self.buzz_lock = False

                if self.alien_type == 'alien_b' and self.buzz_lock:

                    self.current_tim = time.time()
                    


                    i = 0  # Initialize the counter
                    start_time = time.time()  # Record the start time

                    while i < 3:  # Run the loop 4 times
                        current_time = time.time()  # Get the current time

                        if current_time - start_time >= 0.2:
                            self.rc_message.aux3 = 2000
                            self.rc_pub.publish(self.rc_message)


                            start_time = time.time()

                            while time.time() - start_time < 0.2:
                                pass
                            
                            self.rc_message.aux3 = 1000
                            self.rc_pub.publish(self.rc_message)


                            i += 1

                    self.buzz_lock = False

                if self.alien_type == 'alien_c' and self.buzz_lock:

                    self.current_tim = time.time()
                    


                    i = 0  # Initialize the counter
                    start_time = time.time()  # Record the start time

                    while i < 4:  # Run the loop 4 times
                        current_time = time.time()  # Get the current time

                        if current_time - start_time >= 0.2:
                            self.rc_message.aux3 = 2000
                            self.rc_pub.publish(self.rc_message)


                            start_time = time.time()

                            while time.time() - start_time < 0.2:
                                pass
                            
                            self.rc_message.aux3 = 1000
                            self.rc_pub.publish(self.rc_message)


                            i += 1

                    self.buzz_lock = False

                if self.alien_type == 'alien_d' and self.buzz_lock:

                    self.current_tim = time.time()
                    


                    i = 0  # Initialize the counter
                    start_time = time.time()  # Record the start time

                    while i < 5:  # Run the loop 4 times
                        current_time = time.time()  # Get the current time

                        if current_time - start_time >= 0.2:
                            self.rc_message.aux3 = 2000
                            self.rc_pub.publish(self.rc_message)


                            start_time = time.time()

                            while time.time() - start_time < 0.2:
                                pass
                            
                            self.rc_message.aux3 = 1000
                            self.rc_pub.publish(self.rc_message)


                            i += 1

                    self.buzz_lock = False
                
                print(self.maxx_len,self.len_cnts)
                       
                #self.current_waypoint_indx+=2
                self.list_alien.append(self.maxx_len)
                self.maxx_len=0
                
                print('aligned')
                self.next()

    def next(self):
        
        self.clear_block=True
        self.current_waypoint_indx=0
#        self.waypoint=[[0,0,20],[2.25,2.25,22],[4,4,23],[5,5,24],[6,6,25],[8,6.6,26]]
        self.waypoint=[[0,0,20],[0,0,21],[3.8,0,22],[3.8,3.8,23],[3.8,7,24],[7.7,7.7,25],[8.5,8.5,25]]
        self.setpoint=self.waypoint[self.current_waypoint_indx]
        self.pid()


    
        

                                       # PID algorithm
        

    def pid(self):    
        
        print(self.camera_x,self.camera_y)
        
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
            self.error[0] = -(self.drone_whycon_pose_array.poses[0].position.x - self.setpoint[0] )
        # Similarly calculate error for y and z axes 
            
            self.error[1] = (self.drone_whycon_pose_array.poses[0].position.y - self.setpoint[1] )
            self.error[2] = -(self.setpoint[2] - self.drone_whycon_pose_array.poses[0].position.z) 


            if self.earliest_z is None:
                if self.drone_whycon_pose_array.poses[0].position.z:
                    self.Base_z = self.drone_whycon_pose_array.poses[0].position.z 
                    self.node.get_logger().info(f"BASE STATION : {self.Base_z}")
                    self.earliest_z = 1

            

        except:
            pass



        # Calculate derivative and intergral errors. Apply anti windup on integral error (You can use your own method for anti windup, an example is shown here)

        current_time = time.time()
        print(self.rc_message.rc_throttle)
        time_difference = current_time - self.last_time

        self.proportional[0] = (self.Kp[0] * self.error[0])
        self.integral[0] = self.sum_error[0]*self.Ki[0]
        self.derrivative[0] = (self.prev_error[0] - self.error[0])*self.Kd[0]
##
        self.proportional[1] = (self.Kp[1] * self.error[1])
        self.integral[1] = self.sum_error[1]*self.Ki[1]
        self.derrivative[1] = (self.prev_error[1] - self.error[1])*self.Kd[1]

        
            


        self.proportional[2] = (self.Kp[2] * self.error[2])
        self.integral[2] = self.sum_error[2]*self.Ki[2]
        self.derrivative[2] = ((self.prev_error[2] - self.error[2]))*self.Kd[2]
        

        self.last_time = current_time

        self.sum_error[0]=self.sum_error[0] + self.error[0]
        self.sum_error[1]=self.sum_error[1] + self.error[1]
        self.sum_error[2]=self.sum_error[2] + self.error[2]


        if self.integral[0] > SUM_ERROR_ROLL_LIMIT:
            self.integral[0] = SUM_ERROR_ROLL_LIMIT
        if self.integral[0] < -SUM_ERROR_ROLL_LIMIT:
            self.integral[0] = -SUM_ERROR_ROLL_LIMIT
#
        if self.integral[1] > SUM_ERROR_PITCH_LIMIT:
            self.integral[1] = SUM_ERROR_PITCH_LIMIT
        if self.integral[1] < -SUM_ERROR_PITCH_LIMIT:
            self.integral[1] = -SUM_ERROR_PITCH_LIMIT

        if self.integral[2] > SUM_ERROR_THROTTLE_LIMIT:
            self.integral[2] = SUM_ERROR_THROTTLE_LIMIT
        if self.integral[2] < -SUM_ERROR_THROTTLE_LIMIT:
            self.integral[2] = -SUM_ERROR_THROTTLE_LIMIT

        
        # 1 : calculating Error, Derivative, Integral for Pitch error : y axis

        # 2 : calculating Error, Derivative, Integral for Alt error : z axis

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



                                 # BUTTERWORTH FILTER
        

        span = 15
        #for index, val in enumerate([roll, pitch, throttle]):

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
                #self.rc_message.rc_roll = 0
##
            elif index == 1:
                #self.rc_message.rc_pitch = 0
                self.rc_message.rc_pitch = int(filtered_signal[-1])
                #self.rc_message.rc_pitch = 1500

            if index == 2:
                self.rc_message.rc_throttle = int(filtered_signal[-1])
                #self.rc_message.rc_throttle =1450


                

        if self.rc_message.rc_roll > MAX_ROLL:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_roll = MAX_ROLL
        elif self.rc_message.rc_roll < MIN_ROLL:
            self.rc_message.rc_roll = MIN_ROLL
#
        ### Similarly add bounds for pitch yaw and throttle 
        #   
        if self.rc_message.rc_pitch > MAX_PITCH:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_pitch = MAX_PITCH
        elif self.rc_message.rc_pitch < MIN_PITCH:
            self.rc_message.rc_pitch = MIN_PITCH
#
        if self.rc_message.rc_throttle > MAX_THROTTLE:     #checking range i.e. bet 1000 and 2000
            self.rc_message.rc_throttle = MAX_THROTTLE
        elif self.rc_message.rc_throttle < MIN_THROTTLE:
            self.rc_message.rc_throttle = MIN_THROTTLE


        self.rc_msg.rc_roll = int(roll)
        self.rc_msg.rc_pitch = int(pitch)
        self.rc_msg.rc_throttle = int(throttle)
        

        self.rc_pub.publish(self.rc_message)

        self.rc_unfilter.publish(self.rc_msg)


        #if len(self.list_alien)>=2 and self.clear_block==False:
        #    self.next()

        
        
        if self.setpoint == [8.5,8.5,25]:
            if self.at_setpoint():
                print("LANDING FOR FINAL SETPOINT")
                #if self.at_setpoint():
                if self.lock:
                        print("LAND HOVERING")

                        #print(self.lock)
                        #print("inside lock")
                        self.final_throttle=self.rc_message.rc_throttle
                        self.final_roll=self.rc_message.rc_roll
                        self.final_pitch=self.rc_message.rc_pitch
#   
                self.now()
                self.lock = False
        
        print(self.setpoint)   
        print(self.list_alien)     


    # This function will be called as soon as this rosnode is terminated. So we disarm the drone as soon as we press CTRL + C. 
    # If anything goes wrong with the drone, immediately press CTRL + C so that the drone disamrs and motors stop 
    def final(self):
        print("entering final loop")
        self.rc_message.rc_throttle = self.final_throttle
        self.rc_message.rc_roll = self.final_roll
        self.rc_message.rc_pitch = self.final_pitch
        self.now()

    def shutdown_hook(self):
        self.node.get_logger().info("Calling shutdown hook")
        self.disarm()

    # Function to arm the drone 

    def arm(self):
        #self.set_base_z()
        #self.node.get_logger().info("The Base station coordinate is : ", self.Base_z)
        self.node.get_logger().info("Calling arm service")
        self.commandbool.value = True
        self.future = self.arming_service_client.call_async(self.commandbool)

    # Function to disarm the drone 

    def disarm(self):
        self.rc_message.rc_roll=MIN_ROLL
        self.rc_message.rc_pitch=MIN_PITCH
        self.rc_message.rc_throttle=MIN_THROTTLE
        self.node.get_logger().info("Calling DISARM service")
        self.commandbool.value = False
        self.future = self.arming_service_client.call_async(self.commandbool)

        

        # Create the disarm function

    def now(self):
        self.node.get_logger().info("REACHED THE INITIAL HEIGHT : HOVERING STARTED")
        
        self.rc_throttle = 1450
        if self.timer :
            self.hover_start = time.time()
            self.timer = False
            #print("before 10 secs")
        
        if time.time() - self.hover_start > 3:       
            #print("after 10 secs")     
            self.nextt()
        else:
            pass


        





        


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

    #rate = node.create_rate(6)

    try:
        while rclpy.ok():
            #if controller.whycon_data_recieved:
                #controller.pid()
                if node.get_clock().now().to_msg().sec - controller.last_whycon_pose_received_at > 10:
                    node.get_logger().error("Unable to detect WHYCON poses")
                rclpy.spin_once(node) # Sleep for 1/30 secs
                #rate.sleep()
        

    except Exception as err:
        print(err)

    finally:
        controller.shutdown_hook()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()