#!/usr/bin/env python
"""
   pid_velocity - takes messages on wheel_vtarget 
      target velocities for the wheels and monitors wheel for feedback
      
    Copyright (C) 2012 Jon Stephan. 
     
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

    Code updated by John Duarte (2021) - ROS 2 (Foxy)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from common import Wheels
import numpy as np

    
######################################################
######################################################
class PidVelocity(Node):
######################################################
######################################################


    #####################################################
    def __init__(self):
    #####################################################
        super().__init__("pid_velocity")
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)
        
        ### parameters #### 
        self.close_loop = self.declare_parameter('close_loop', True).value
        self.kp = self.declare_parameter('kp',10).value
        self.ki = self.declare_parameter('ki',10).value
        self.kd = self.declare_parameter('kd',0.001).value

        self.limited = self.declare_parameter('limited', True).value
        self.vel_min = self.declare_parameter('vel_min',-1.0).value * np.ones(2)
        self.vel_max = self.declare_parameter('vel_max',1.0).value * np.ones(2)

        self.get_logger().debug("%s got kp:%0.3f ki:%0.3f kd:%0.3f" % (self.nodename, self.kp, self.ki, self.kd))

        #### subscribers/publishers 
        self.create_subscription(Wheels, "goal_wheels", self.goalWheelsCallback, qos_profile_system_default)
        self.create_subscription(Wheels, "cal_wheels", self.calWheelsCallback, qos_profile_system_default)
 
        self.pub_motor = self.create_publisher(Wheels, 'cmd_wheels', qos_profile_system_default) 

        self.create_timer(5, self.parametersCallback())
            
    #####################################################
    def init(self):
    #####################################################
        ### initialize variables
        self.prev_pid_time = self.get_clock().now()
        self.previous_error = np.zeros(2)
        self.integral = np.zeros(2)
        self.target = np.zeros(2)
        self.sensor = np.zeros(2)
    #####################################################
    def update(self):
    #####################################################
        now = self.get_clock().now()
        self.doPid(now)
        self.pub_motor.publish(self.motor)
        
    #####################################################
    def doPid(self, now):
    #####################################################
        
        # Time interval
        pid_dt_duration = now - self.prev_pid_time
        pid_dt = pid_dt_duration.to_sec()
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
        motor_limited = np.clip(motor, self.vel_min, self.vel_max)
        if motor_limited != motor:
            self.integral = self.integral - (error * pid_dt)
      
        if ( all(self.target == 0) ):
            motor = motor * 0.0
            motor_limited = motor_limited * 0.0
        
        if self.limited:
            self.publishCmdWheels(motor_limited)
        else:
            self.publishCmdWheels(motor)
    
    #####################################################
    def goalWheelsCallback(self,msg):
    #####################################################
        self.target = msg.param
        if not self.close_loop:
            self.update()

    #####################################################
    def calWheelsCallback(self, msg):
    #####################################################
        self.sensor = msg.param
        if self.close_loop:
            self.update()

    #####################################################
    def publishCmdWheels(self, motor):
    #####################################################
        cmd_wheels = Wheels()
        cmd_wheels.param[0] = motor[0]
        cmd_wheels.param[1] = motor[1]

        self.pub_motor.publish(cmd_wheels)
    
    #####################################################
    def parametersCallback(self):
    #####################################################
        self.close_loop = self.get_parameter('close_loop').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        self.limited = self.get_parameter('limited').value
        self.vel_min = self.get_parameter('vel_min').value * np.ones(2)
        self.vel_max = self.get_parameter('vel_max').value * np.ones(2)

def main(args=None):
    ##########################################################################
    ##########################################################################
    rclpy.init(args=args)

    node = PidVelocity()
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
