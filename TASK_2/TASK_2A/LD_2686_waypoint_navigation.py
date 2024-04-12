#!/usr/bin/env python3

# Importing the required libraries
from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time

class swift():
    def __init__(self):
        rospy.init_node('drone_control')

        self.hover_duration = 2.0  # Time to hover at each waypoint in seconds
        self.hover_start_time = time.time()
        self.in_hover_mode = False

        self.drone_position = [0.0, 0.0, 0.0]

        self.waypoint = [
            [0, 0, 23],
            [2, 0, 23],
            [2, 2, 23],
            [2, 2, 25],
            [-5, 2, 25],
            [-5, -3, 25],
            [-5, -3, 21],
            [7, -3, 21],
            [7, 0, 21],
            [0, 0, 19]
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
        self.Kd = [186, 104.7, 789.9]

        self.error = [0, 0, 0]
        self.prev_error = [0, 0, 0]
        self.sum_error = [0, 0, 0]
        self.faci = 0
        self.facd = 0

        self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
        self.throttle_error_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
        self.pitch_error_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
        self.roll_error_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)

        rospy.Subscriber('/whycon/poses', PoseArray, self.whycon_callback)
        rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
        rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitchy_set_pid)
        rospy.Subscriber('/pid_tuning_roll', PidTune, self.rollx_set_pid)

        self.arm()

    def disarm(self):
        self.cmd.rcAUX4 = 1100
        self.command_pub.publish(self.cmd)
        rospy.sleep(1)

    def arm(self):
        self.disarm()
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
        x_tolerance = 0.3
        y_tolerance = 0.3
        z_tolerance = 0.3
        x_at_setpoint = abs(self.setpoint[0] - self.drone_position[0]) <= x_tolerance
        y_at_setpoint = abs(self.setpoint[1] - self.drone_position[1]) <= y_tolerance
        z_at_setpoint = abs(self.setpoint[2] - self.drone_position[2]) <= z_tolerance
        return x_at_setpoint and y_at_setpoint and z_at_setpoint

    def pid(self):
        if self.current_waypoint_indx < len(self.waypoint):
            if self.at_setpoint() and not self.in_hover_mode:
                self.hover_start_time = time.time()
                self.in_hover_mode = True
            elif self.in_hover_mode and (time.time() - self.hover_start_time) >= self.hover_duration:
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

        # Rest of the PID controller code remains the same as before

        self.error[2] = -(self.setpoint[2] - self.drone_position[2])
        self.cmd.rcThrottle = int(1585 + (self.error[2] * self.Kp[2]) + (self.error[2] - self.prev_error[2]) * self.Kd[2] + (self.sum_error[2] * self.Ki[2]))
        if self.cmd.rcThrottle > 2000:
            self.cmd.rcThrottle = 2000
        elif self.cmd.rcThrottle < 1000:
            self.cmd.rcThrottle = 1000
        self.prev_error[2] = self.error[2]
        self.sum_error[2] = self.sum_error[2] + self.error[2]

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
        self.cmd.rcPitch = int(1500 + (self.error[1] * self.Kp[1]) + (self.error[1] - self.prev_error[1]) * self.Kd[1] + (self.sum_error[1] * self.Ki[1]))
        if self.cmd.rcPitch > 2000:
            self.cmd.rcPitch = 2000
        elif self.cmd.rcPitch < 1000:
            self.cmd.rcPitch = 1000
        self.prev_error[1] = self.error[1]
        self.sum_error[1] = self.sum_error[1] + self.error[1]

        self.error[0] = self.setpoint[0] - self.drone_position[0]
        self.cmd.rcRoll = int(1500 + (self.error[0] * self.Kp[0]) + (self.error[0] - self.prev_error[0]) * self.Kd[0] + (self.sum_error[0] * self.Ki[0]))
        if self.cmd.rcRoll > 2000:
            self.cmd.rcRoll = 2000
        elif self.cmd.rcRoll < 1000:
            self.cmd.rcRoll = 1000
        self.prev_error[0] = self.error[0]
        self.sum_error[0] = self.sum_error[0] + self.error[0]

        self.command_pub.publish(self.cmd)
        self.throttle_error_pub.publish(self.error[2])
        self.pitch_error_pub.publish(self.error[1])
        self.roll_error_pub.publish(self.error[0])

if __name__ == '__main__':
    e_drone = swift()
    r = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            e_drone.pid()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
