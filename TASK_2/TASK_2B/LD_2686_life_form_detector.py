#!/usr/bin/env python3

# Importing the required libraries
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

        self.hover_duration = 1.0  # Time to hover at each waypoint in seconds
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
        self.clear_block=False


        

        self.drone_position = [0.0, 0.0, 0.0]

        self.waypoint = [[0,0,26],
                         
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
            [-9,2.25,26],
            [-9,0,26],
            [-9,2-.25,26],
            [-9,-4.5,26],
            [-9,-6.75,26],
            [-9,-9,26],
                         
            
            
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
            [-9,9,26],
            [-9,6.75,26],
            [-9,4.5,26],
            [-6.75,6.75,26],
            [-6.75,4.5,26],
            [-6.75,2.25,26],
            [-6.75,0,26],
            [-6.75,-2.25,26],
            [-6.75,-4.5,26],
            [-6.75,-6.75,26],

            

            [-2.25,-4.5,26],
            [0,-4.5,26],
            [2.25,-4.5,26],
            [4.5,-4.5,26],
            [4.5,-2.5,26],
            [4.5,0,26],
            [4.5,2.5,26],
            [4.5,4.5,26],
            [2.25,4.5,26],
            [0,4.5,26],
            [-2.25,4.5,26],
            [-4.5,4.5,26],
            [-4.5,2.25,26],
            [-4.5,0,26],
            [-4.5,-2.25,26],
            [0,-2.25,26],
            [2.25,-2.25,26],
            [2.25,0.0,26],
            [2.25,2.25,26],
            [0,2.25,26],
            [-2.25,2.25,26],
            [-2.25,0,26],
            [-2.25,-2.25,26]]
        
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
        #self.disarm()
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
        x_tolerance = 0.1
        y_tolerance = 0.1
        z_tolerance = 0.15
        x_at_setpoint = abs(self.setpoint[0] - self.drone_position[0]) <= x_tolerance
        y_at_setpoint = abs(self.setpoint[1] - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs(self.setpoint[2] - self.drone_position[2]) <= z_tolerance
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    
    def at_new_setpoint(self):
        x_tolerance = 0.1
        y_tolerance = 0.1
        z_tolerance = 0.15
        x_at_setpoint = abs(self.new_setpoint[0] - self.drone_position[0]) <= x_tolerance
        y_at_setpoint = abs(self.new_setpoint[1] - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs(self.new_setpoint[2] - self.drone_position[2]) <= z_tolerance
        return x_at_setpoint and y_at_setpoint and z_at_setpoint
    
    def image_callback(self, data):
        self.desired_camera_x=0
        self.desired_camera_y=0
        
        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)
        thresh = cv2.threshold(blurred, 180, 255, cv2.THRESH_BINARY)[1]
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
            
        #print(self.centroid_list)
        if len(cnts) ==3 and self.clear_block==False:
            self.waypoint=self.waypoint[0:self.current_waypoint_indx]
            #self.waypoint.append([11,11,25])
            
        

        if self.len_cnts > self.max_len:
            self.max_len=self.len_cnts

        if self.max_len==2:
            self.alien_type='alien_a'
        elif self.max_len==3:
            self.alien_type='alien_b'
        elif self.max_len==4:
            self.alien_type='alien_c'
            

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
        #print(self.camera_x,self.camera_y)
        self.error_x = self.desired_camera_x - self.camera_x
        self.error_y = self.desired_camera_y - self.camera_y

    def navigate_to_target(self):
        if not self.in_hover_mode:
            self.setpoint = self.target_position

        if not self.at_setpoint():
            self.pid()  # Continue using the existing PID control to reach the target

    # Once at the target position, enable hovering
        if self.at_setpoint() and not self.in_hover_mode:
            self.in_hover_mode = True
            self.hover_start_time = time.time()
    
    # Hover for the specified duration
        if self.in_hover_mode and (time.time() - self.hover_start_time) >= self.hover_duration:
            self.in_hover_mode = False
            self.disarm()

    def aligning(self):

        
            if abs(self.camera_x)>0.01 and abs(self.camera_y)>0.01:
                self.setpoint[0]=self.drone_position[0]+self.camera_x
                self.setpoint[1]=self.drone_position[1]+self.camera_y
                
            else:
                self.biolocation_msg.organism_type = self.alien_type  
                self.biolocation_msg.whycon_x = self.drone_position[0]+self.camera_x
                self.biolocation_msg.whycon_y = self.drone_position[1]+self.camera_y
                self.biolocation_msg.whycon_z = 37.63
                print(self.camera_x)
                print(self.camera_y)
                #self.waypoint.clear
                self.next()
        
            
            #self.biolocation_publisher.publish(self.biolocation_msg)
            #print(self.camera_x,self.camera_y)

            #step_size = 1  # You can adjust the step size as needed

    # Gradually move towards the target position
            #while self.setpoint[0] < 11.0 or self.setpoint[1] < 11.0 or self.setpoint[2] < 25.0:
        # Gradually increase the setpoint
                #self.setpoint[0] += step_size
                #self.setpoint[1] += step_size
                #self.setpoint[2] += step_size
                #self.pid()  # Use your existing PID control to move the drone

        # Sleep briefly to control the speed of movement
                #time.sleep(0.1)

    # After reaching the target position, hover for 10 seconds
            #self.in_hover_mode = True
            #self.hover_start_time = time.time()
            #while time.time() - self.hover_start_time < 10.0:
            #    self.pid()  # Continue using PID control for hovering

    # Disarm after hovering
            #self.in_hover_mode = False
            #self.disarm()
            #print('DISARMED')
            #self.hover_start_time=time.time()
            #while time.time()-self.hover_start_time <10:
            #    self.in_hover_mode=True
            #    pass
            #else:
            #self.setpoint=[10.5,10.5,25]
            #self.hover_start_time=time.time()
            #if time.time()-self.hover_start_time >=5:
            #print('ENTERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR')

            #self.disarm()
                #self.in_hover_mode=False
                #self.hover_start_time=time.time()
                #while time.time()-self.hover_start_time <10:
                #    self.in_hover_mode=True
                #    pass
                #else:
                #    self.disarm() 


    def hover(self, duration):
        if self.start_time is None:
            self.start_time = time.time()

        current_time = time.time()
        elapsed_time = current_time - self.start_time

        if elapsed_time < duration:
            # Continue hovering
            pass
        else:
            # Hovering duration reached, stop hovering
            self.start_time = None


    def next(self):
        #self.setpoint = [5, 5, 20]
        #time.sleep(5)
        #self.disarm()

        #self.setpoint = [5, 5, 20]
        self.clear_block=True
        self.current_waypoint_indx=0
        self.waypoint=[[2.25,2.25,30],[6.75,6.75,35],[8,8,36],[11.1,11,37.2],[11.1,11,37.5]]
        self.setpoint=self.waypoint[self.current_waypoint_indx]
        self.pid()
        #print(self.current_waypoint_indx,self.waypoint)

        
        

    # Drone has reached the setpoint, now disarm
        #self.disarm()
    
        

    #def found_aliens(self):
                #self.waypoint.clear
                #self.setpoint.clear
                #self.new_setpoint=[8,8,25]
                #print(self.new_setpoint)

                #if self.at_new_setpoint():
                #    print('YAHALO')
                #    self.disarm()
                #self.navigate_to_target()
                #self.setpoint=[11,11,25]
                #self.pid()
    # Set the new setpoint to the desired coordinates
                
                #self.new_setpoint = [11, 11, 25]

    # Fly to the desired position using the PID controller
                #while not self.at_new_setpoint():
                #    self.pid()

    # Once the drone is at the desired position, disarm it
                #self.disarm()

                #new_waypoint=[[2.25,2.25,30],[4.5,4.5,30],[6.75,6.75,30],[9,9,30],[11,11,30]]
                #for curr_index in range(len(new_waypoint)):

                    #if curr_index < len(new_waypoint)-1:
                        #print('first looooooop')
                        #self.new_setpoint=new_waypoint[curr_index]
                #self.hover_start_time=time.time()
                #while time.time()-self.hover_start_time <2.0:
                #    pass
                #else:
                #    self.in_hover_mode=False
                #    self.disarm()
                    #self.landing()

            
    #def landing(self):
            #self.new_setpoint=[11,11,25]
    #        print("entered the loop of landing")
    #        self.waypoint.clear
            #self.waypoint=[11,11,25]



            #if self.at_new_setpoint():
            #self.disarm()
            #if not self.in_hover_mode:
            #    self.new_setpoint=[11,11,30]
            #    print(self.new_setpoint)
            #    self.hover_start_time = time.time()
                #self.in_hover_mode = True

            #elif self.in_hover_mode and (time.time() - self.hover_start_time) >= 3.0:  # Hover for 5 seconds
            #    print('non hover modeeeeeee')
            #    while time.time()-self.hover_start_time<5.0:
            #        print('entered the while loop')
            #        pass
            #    else:
            #        self.in_hover_mode = False
            #        self.setpoint.clear
                    #ospy.signal_shutdown("LANDING COMPLETE")
    
    #def naviagtion(self):

                
                
            
		
            

    def pid(self):


        #if self.in_hover_mode and (time.time() - self.hover_start_time) >= self.hover_duration:
        #    self.in_hover_mode = False
        #    self.current_waypoint_indx += 1
        #    if self.current_waypoint_indx < len(self.waypoint):
        #        self.setpoint = self.waypoint[self.current_waypoint_indx]

        #print('NIGAAAAAAAAAAA')
        if self.current_waypoint_indx < len(self.waypoint):
            if self.at_setpoint() and not self.in_hover_mode:
                self.hover_start_time = time.time()
                self.in_hover_mode = True
            elif self.in_hover_mode and (time.time() - self.hover_start_time) >= 1:
                self.in_hover_mode = False
                self.current_waypoint_indx += 1
                if self.current_waypoint_indx < len(self.waypoint):
                    self.setpoint = self.waypoint[self.current_waypoint_indx]
                    
        #print()

        if 0.15 < self.error[2] < 7:
            self.cmd.rcThrottle = 1585
        if -7 < self.error[2] < -0.15:
            self.cmd.rcThrottle = 1585
        if 0.15 < self.error[2] < -0.1:
            self.cmd.rcThrottle = 1500

        

        # Rest of the PID controller code remains the same as before
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
        
        if self.setpoint not in [[11,10.9,36.5]]:
            if self.current_waypoint_indx == len(self.waypoint):
                if abs(self.error[0]) <= 0.03 and abs(self.error[1]) <= 0.03:
                    self.in_hover_mode = True
                    self.hover_start_time = time.time()
                    if time.time()-self.in_hover_mode >=3:
                        self.in_hover_mode=False
                        self.stop_pid=True
                        self.aligning()
        

        if self.setpoint==[11.1,11,37.5]:
            self.disarm()
        
            #self.disarm()

                #if self.in_hover_mode and time.time() - self.hover_start_time >= 5.0:
                #    print('shinaaee')
                #    self.in_hover_mode = False
                #    self.aligning()
        #print(self.error_x,self.error_y)
    
    
            


if __name__ == '__main__':
    
    e_drone = swift()
    r = rospy.Rate(30)

    def on_shutdown():
        e_drone.biolocation_publisher.publish(e_drone.biolocation_msg)
    rospy.on_shutdown(on_shutdown)



    try:
        while not rospy.is_shutdown():
            e_drone.pid()
            r.sleep()

    except rospy.ROSInterruptException:

        pass



