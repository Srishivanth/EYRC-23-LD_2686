#!/usr/bin/env python3

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from std_msgs.msg import Int16, Int64
from skimage import measure
import numpy as np
import imutils
import cv2
from geometry_msgs.msg import PoseArray
from luminosity_drone.msg import Biolocation
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
from threading import Timer


class swift():
    def __init__(self):
        rospy.init_node('drone_control')

        self.hover_duration = 1.0  
        self.hover_start_time = time.time()
        self.in_hover_mode = False
        self.bridge=CvBridge()
        self.centroid_list=[]
        self.alien_type=''
        self.biolocation_msg=Biolocation()
        self.biolocation_msg.organism_type=''
        self.biolocation_msg.whycon_x=0.0
        self.biolocation_msg.whycon_y=0.0
        self.biolocation_msg.whycon_z=0.0
        self.fx=1233.2887188967234
        self.fy=1233.2887188967234
        self.cx=250.5
        self.cy=250.5
        self.camera_x=0.0
        self.camera_y=0.0
        self.error_x=0
        self.error_y=0
        self.new_error=[0.0,0.0,0.0]
        self.new_prev_error=[0.0,0.0,0.0]
        self.new_sum_error=[0.0,0.0,0.0]
        self.new_setpoint=[0,0,0]
        self.new_len=0
        self.len_cnts=0
        self.max_len=0
        self.final=[11,11,25]
        self.target_position = [11, 11, 25]
        self.stop_pid=False
        self.start_time = None
        self.list_alien=[0.5]
        self.clear_block=False
        self.maxx_len=0


        

        self.drone_position = [0.0, 0.0, 0.0]

        self.waypoint = [[0,0,26],
                         
            [-4.5,0,26],
            [-4.5,-2.25,26],
            
            [-4.5,-4.5,26],
            [-4.5,-6.75,26],
            [-2.25,-6.75,26],
            [0,-6.75,26],
            [2.25,-6.75,26],
            [4.5,-6.75,26],
            [6.75,-6.75,26],
            [6.75,-4.5,26],
            [6.75,-2.25,26],
            [6.75,0,26],
            [6.75,2.25,26],
            [6.75,4.5,26],
            [6.75,6.75,26],
            [4.5,6.75,26],
            [2.25,6.75,26],
            [0,6.75,26],
            [-2.25,6.75,26],
            [-4.5,6.75,26],
            [-6.75,9,26],
            
                         
            [-6.75,-9,26],
            [-4.5,-9,26],
            [-2.25,-9,26],
            [0,-9,26],
            [2.25,-9,26],
            [4.5,-9,26],
            [6.75,-9,26],
            [9,-9,26],
            [9,-6.75,26],
            [9,-4.5,26],
            [9,-2.25,26],
            [9,0,26],
            [9,2.25,26],
            [9,4.5,26],
            [9,6.75,26],
            [9,9,26],
            [6.75,9,26],
            [4.5,9,26],
            [2.25,9,26],
            [0,9,26],
            [-2.25,9,26],
            [-4.5,9,26],
            [-6.75,9,26],
            [-9,9,26],
            [-9,2.25,26],
            [-9,0,26],
            [-9,2-.25,26],
            [-9,-4.5,26],
            [-9,-6.75,26],
            [-9,-9,26],
                         
            
            
            
            [-6.75,6.75,26],
            [-6.75,4.5,26],
            [-6.75,2.25,26],
            [-6.75,0,26],
            [-6.75,-2.25,26],
            [-6.75,-4.5,26],
            [-6.75,-6.75,26],

            

            #[-2.25,-4.5,26],
            #[0,-4.5,26],
            #[2.25,-4.5,26],
            #[4.5,-4.5,26],
            #[4.5,-2.5,26],
            #[4.5,0,26],
            #[4.5,2.5,26],
            #[4.5,4.5,26],
            #[2.25,4.5,26],
            #[0,4.5,26],
            #[-2.25,4.5,26],
            #[-4.5,4.5,26],
            #[-4.5,2.25,26],
            #[-4.5,0,26],
            #[-4.5,-2.25,26],
            #[0,-2.25,26],
            #[2.25,-2.25,26],
            #[2.25,0.0,26],
            #[2.25,2.25,26],
            #[0,2.25,26],
            #[-2.25,2.25,26],
            #[-2.25,0,26],
            #[-2.25,-2.25,26]
            ]
        
        self.current_waypoint_indx = 0
        self.setpoint = self.waypoint[self.current_waypoint_indx]

        self.cmd = swift_msgs()
        self.cmd.rcRoll = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX1 = 1500
        self.cmd.rcAUX2 = 1500
        self.cmd.rcAUX3 = 1500
        self.cmd.rcAUX4 = 1500

        self.Kp = [20.5, 20.62, 29.18]
        self.Ki = [0, 0, 0]
        self.Kd = [386, 304.7, 789.9]

        self.error = [0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0]
        self.sum_error = [0.0, 0.0, 0.0]
        self.faci = 0
        self.facd = 0

        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.throttle_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
        self.biolocation_publisher = rospy.Publisher('astrobiolocation', Biolocation, queue_size=10)

        rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitchy_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.rollx_set_pid)
        rospy.Subscriber('/swift/camera_rgb/image_raw', Image, self.image_callback)


        self.arm()

    def disarm(self):
        self.cmd.rcAUX4 = 1000
        self.cmd.rcThrottle=1000
        self.cmd.rcRoll=1000
        self.cmd.rcPitch=1000
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)
        print('reached disarm')
        rospy.signal_shutdown('DISARMED')

    def arm(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1585
        self.cmd.rcAUX4 = 1500
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.Kp * 0.06
        self.Ki[2] = alt.Ki * 0.008
        self.Kd[2] = alt.Kd * 0.3

    def pitchy_set_pid(self, alt):
        self.Kp[1] = alt.Kp * 0.06
        self.Ki[1] = alt.Ki * 0.008
        self.Kd[1] = alt.Kd * 0.3

    def rollx_set_pid(self, alt):
        self.Kp[0] = alt.Kp * 0.06
        self.Ki[0] = alt.Ki * 0.008
        self.Kd[0] = alt.Kd * 0.3

    def at_setpoint(self):
        x_tolerance = 0.2
        y_tolerance = 0.2
        z_tolerance = 0.2
        x_at_setpoint = abs(self.setpoint[0] - self.drone_position[0]) <= x_tolerance
        y_at_setpoint = abs(self.setpoint[1] - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs(self.setpoint[2] - self.drone_position[2]) <= z_tolerance
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    
    
    def image_callback(self, data):
        self.desired_camera_x=0
        self.desired_camera_y=0
        
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 160, 255, cv2.THRESH_BINARY)[1]
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
            
        if len(cnts) in [2,3,4] and self.clear_block==False:
            if len(cnts) not in self.list_alien:
                    print('hi')
                    self.in_hover_mode = True
                    self.hover_start_time = time.time()
                    if time.time()-self.in_hover_mode >=1:
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

        
            if abs(self.camera_x)>0.00199999 and abs(self.camera_y)>0.00199999:
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

                #if self.at_setpoint():
                #    self.cmd.rcThrottle=1500
                #    self.prev_error=[0,0,0]
                #    self.in_hover_mode = True
                #    self.hover_start_time = time.time()
                #    if time.time()-self.in_hover_mode >=1:
                #        self.in_hover_mode=False
                self.biolocation_msg.organism_type = self.alien_type  
                self.biolocation_msg.whycon_x = self.setpoint[0]#self.drone_position[0]+self.camera_x
                self.biolocation_msg.whycon_y = self.setpoint[1]#self.drone_position[1]+self.camera_y
                self.biolocation_msg.whycon_z = 37.63
                self.biolocation_publisher.publish(self.biolocation_msg)
                
                print(self.maxx_len,self.len_cnts)
                       
                self.current_waypoint_indx+=2
                self.list_alien.append(self.maxx_len)
                self.maxx_len=0
                
                print('aligned')
        
            

    def next(self):
        
        self.clear_block=True
        self.current_waypoint_indx=0
        self.waypoint=[[2.25,2.25,30],[6.75,6.75,35],[8,8,36],[11.1,11,37.2],[11.1,11,37.5]]
        self.setpoint=self.waypoint[self.current_waypoint_indx]
        self.pid()


    def pid(self):


       
        if self.current_waypoint_indx < len(self.waypoint):
            if self.at_setpoint() and not self.in_hover_mode:
                self.hover_start_time = time.time()
                self.in_hover_mode = True
            elif self.in_hover_mode and (time.time() - self.hover_start_time) >= 2:
                self.in_hover_mode = False
                self.current_waypoint_indx += 1
                if self.current_waypoint_indx < len(self.waypoint):
                    self.setpoint = self.waypoint[self.current_waypoint_indx]
                    
        if 0.15 < self.error[2] < 7:
            self.cmd.rcThrottle = 1585
        if -7 < self.error[2] < -0.15:
            self.cmd.rcThrottle = 1585
        if 0.15 < self.error[2] < -0.1:
            self.cmd.rcThrottle = 1500

        
        self.new_error[2] = -(self.new_setpoint[2] - self.drone_position[2])

        self.error[2] = -(self.setpoint[2] - self.drone_position[2])

        self.cmd.rcThrottle = int(1585 + (self.new_error[2] * self.Kp[2]) + (self.new_error[2] - self.new_prev_error[2]) * self.Kd[2] + (self.new_sum_error[2] * self.Ki[2]))

        self.cmd.rcThrottle = int(1585 + (self.error[2] * self.Kp[2]) + (self.error[2] - self.prev_error[2]) * self.Kd[2] + (self.sum_error[2] * self.Ki[2]))
        if self.cmd.rcThrottle > 2000:
            self.cmd.rcThrottle = 2000
        elif self.cmd.rcThrottle < 1000:
            self.cmd.rcThrottle = 1000
        self.prev_error[2] = self.error[2]
        self.sum_error[2] = self.sum_error[2] + self.error[2]

        self.new_prev_error[2] = self.new_error[2]
        self.new_sum_error[2] = self.new_sum_error[2] + self.new_error[2]

        self.faci = (self.sum_error[2] * self.Ki[2])

        if (self.cmd.rcThrottle > 1505 and self.cmd.rcThrottle < 1495) and (self.error[2] * self.faci > 0):
            self.Ki[2] = 0.0
        else:
            self.Ki[2] = 0.1
        if self.sum_error[2] > 0.05:
            self.sum_error[2] = 0.05
        elif self.sum_error[2] < -0.05:
            self.sum_error[2] = -0.05

        self.error[1] = -(self.setpoint[1] - self.drone_position[1])

        self.new_error[1] = -(self.new_setpoint[1] - self.drone_position[1])

        self.cmd.rcPitch = int(1500 + (self.new_error[1] * self.Kp[1]) + (self.new_error[1] - self.new_prev_error[1]) * self.Kd[1] + (self.new_sum_error[1] * self.Ki[1]))


        self.cmd.rcPitch = int(1500 + (self.error[1] * self.Kp[1]) + (self.error[1] - self.prev_error[1]) * self.Kd[1] + (self.sum_error[1] * self.Ki[1]))
        if self.cmd.rcPitch > 2000:
            self.cmd.rcPitch = 2000
        elif self.cmd.rcPitch < 1000:
            self.cmd.rcPitch = 1000
        self.prev_error[1] = self.error[1]
        self.sum_error[1] = self.sum_error[1] + self.error[1]

        self.new_prev_error[1] = self.new_error[1]
        self.new_sum_error[1] = self.new_sum_error[1] + self.new_error[1]

        self.error[0] = self.setpoint[0] - self.drone_position[0]

        self.new_error[0] = self.new_setpoint[0] - self.drone_position[0]

        self.cmd.rcRoll = int(1500 + (self.new_error[0] * self.Kp[0]) + (self.new_error[0] - self.new_prev_error[0]) * self.Kd[0] + (self.new_sum_error[0] * self.Ki[0]))


        self.cmd.rcRoll = int(1500 + (self.error[0] * self.Kp[0]) + (self.error[0] - self.prev_error[0]) * self.Kd[0] + (self.sum_error[0] * self.Ki[0]))
        if self.cmd.rcRoll > 2000:
            self.cmd.rcRoll = 2000
        elif self.cmd.rcRoll < 1000:
            self.cmd.rcRoll = 1000
        self.prev_error[0] = self.error[0]
        self.sum_error[0] = self.sum_error[0] + self.error[0]

        self.new_prev_error[0] = self.new_error[0]
        self.new_sum_error[0] = self.new_sum_error[0] + self.new_error[0]

        self.command_pub.publish(self.cmd)
        self.throttle_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])
        
        if len(self.list_alien)==4 and self.clear_block==False:
            self.next()

     

        if self.setpoint==[11.1,11,37.5]:
            self.disarm()
 
            


if __name__ == '__main__':
    
    e_drone = swift()
    r = rospy.Rate(30)

    #def on_shutdown():
    #    e_drone.biolocation_publisher.publish(e_drone.biolocation_msg)
    #rospy.on_shutdown(on_shutdown)



    try:
        while not rospy.is_shutdown():
            e_drone.pid()
            r.sleep()

    except rospy.ROSInterruptException:

        pass



