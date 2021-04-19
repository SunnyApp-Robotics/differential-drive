#!/usr/bin/env python
"""
    Kinematic (twist_to_motors) - converts a twist message to motor commands.  Needed for navigation stack
    
    Copyright (C) 2012 Jon Stephan. 

    Code updated by John Duarte (2021) - ROS 2 (Foxy)
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Twist
from common.msg import Wheels
import numpy as np

#############################################################
#############################################################


class Kinematics(Node):
    #############################################################
    #############################################################

    #############################################################
    def __init__(self):
        #############################################################
        super().__init__("kinematics")
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)

        #### parameters kinematics #######
        self.radius = float(self.declare_parameter(
            'wheels.radius', 0.02569).value)  # The wheel radius in meters
        self.base_width = float(self.declare_parameter(
            'wheels.base_width', 0.1275).value)  # The wheel base width in meters

        ### parameters control ####
        self.close_loop = self.declare_parameter('close_loop', True).value
        self.kp = self.declare_parameter('kp', 10).value
        self.ki = self.declare_parameter('ki', 10).value
        self.kd = self.declare_parameter('kd', 0.001).value

        self.limited = self.declare_parameter('limited', True).value
        self.linear_max = self.declare_parameter('linear_max', 0.621).value
        self.angular_max = self.declare_parameter('angular_max', 4.84).value

        self.get_logger().debug("%s got kp:%0.3f ki:%0.3f kd:%0.3f" %
                                (self.nodename, self.kp, self.ki, self.kd))

        # Init variables
        self.init()

        # subscriptions / publishers
        self.create_subscription(
            Twist, 'cmd_vel', self.cmdVelCallback, qos_profile_system_default)
        self.create_subscription(
            Twist, "cal_vel", self.calVelCallback, qos_profile_system_default)

        self.cmd_wheels_pub = self.create_publisher(
            Wheels, 'robot/cmd_wheels', qos_profile_system_default)

        # 5 seconds timer to update parametersl
        self.create_timer(5, self.parametersCallback)

    #############################################################################
    def init(self):
        #############################################################################

        # initialize variables
        self.prev_pid_time = self.get_clock().now()
        self.previous_error = np.zeros(2)
        self.integral = np.zeros(2)

        self.cmd_wheels = np.zeros(2)

        self.target = np.zeros(2)
        self.sensor = np.zeros(2)

    #############################################################
    def update(self):
        #############################################################
        now = self.get_clock().now()  # Current time
        self.doPid(now)
        self.publishWheelsCmd(now)

    #####################################################
    def doPid(self, now):
        #####################################################

        if self.close_loop:
            # Time interval
            pid_dt_duration = now.nanoseconds - self.prev_pid_time.nanoseconds
            pid_dt = float(pid_dt_duration) / 1e9
            self.prev_pid_time = now

            # Error
            error = np.array(self.target - self.sensor)

            # Proportional
            errorKp = self.kp * error

            # Integral
            self.integral = self.integral + (error * pid_dt)
            errorKi = self.ki * self.integral

            # Derivative
            derivative = (error - self.previous_error) / pid_dt
            self.previous_error = error
            errorKd = self.kd * derivative

            # PID
            motor = (errorKp) + (errorKi) + (errorKd)

            # Limiter
            motor_limited = np.clip(motor, -np.array([self.linear_max, self.angular_max]), np.array([self.linear_max, self.angular_max]))
            if motor_limited != motor:
                self.integral = self.integral - (error * pid_dt)

            if (all(self.target == 0)):
                motor = motor * 0.0
                motor_limited = motor_limited * 0.0

            if self.limited:
                self.calculateKinematics(motor_limited)
            else:
                self.calculateKinematics(motor)
        
        else:
            self.calculateKinematics(self.target)

    #############################################################
    def calculateKinematics(self, cmd_vel):
    #############################################################
        dx = cmd_vel[0]
        dr = cmd_vel[1]

        left = (dx - self.base_width * dr) / self.radius
        right = (dx + self.base_width * dr) / self.radius

        self.cmd_wheels[0] = right
        self.cmd_wheels[1] = left

    #############################################################
    def publishWheelsCmd(self):
        #############################################################
        cmd_wheels = Wheels()
        cmd_wheels.param[0] = self.cmd_wheels[0]
        cmd_wheels.param[1] = self.cmd_wheels[1]

        self.cmd_wheels_pub.publish(cmd_wheels)

    #############################################################
    def cmdVelCallback(self, msg):
        #############################################################
        self.target[0] = msg.linear.x
        self.target[1] = msg.angular.z

        if not self.close_loop:
            self.update()

    #####################################################
    def calVelCallback(self, msg):
        #####################################################
        self.sensor[0] = msg.linear.x
        self.sensor[1] = msg.angular.z

        if self.close_loop:
            self.update()

    #############################################################
    def parametersCallback(self):
        #############################################################
        # The wheel radius in meters
        self.radius = float(self.get_parameter('wheels.radius').value)
        # The wheel base width in meters
        self.base_width = float(self.get_parameter('wheels.base_width').value)

        self.close_loop = self.get_parameter('close_loop').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.limited = self.get_parameter('limited').value
        self.linear_max = self.get_parameter('linear_max').value
        self.angular_max = self.get_parameter('angular_max').value


def main(args=None):
    ##########################################################################
    ##########################################################################
    rclpy.init(args=args)

    node = Kinematics()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown(0)


#############################################################################
#############################################################################
if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass
