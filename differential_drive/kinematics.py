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
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from common import Wheels

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

        #### parameters #######
        self.radius = float(self.declare_parameter('wheels.radius', 0.012).value) # The wheel radius in meters
        self.base_width = float(self.declare_parameter('wheels.base_width', 0.245).value) # The wheel base width in meters
        
        # Init variables
        self.init()

        # subscriptions / publishers
        self.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback, qos_profile_system_default)
        self.cmd_wheels_pub = self.create_publisher(Wheels, 'goal_wheels', qos_profile_system_default)

        # 5 seconds timer to update parameters
        self.create_timer(5, self.parametersCallback)
    
    #############################################################################
    def init(self):
    #############################################################################
        self.left = 0
        self.right = 0

    #############################################################
    def update(self):
    #############################################################
        now = self.get_clock().now() #Current time
        self.calculateKinematics(now, self.cmd_dx, self.cmd_dr)

    #############################################################
    def calculateKinematics(self, now, dx, dr):
    #############################################################
        self.left = dx - self.base_width * dr / self.radius
        self.right = dx + self.base_width * dr / self.radius

        self.publishWheelsCmd(now)

    #############################################################
    def publishWheelsCmd(self, now):
    #############################################################
        cmd_wheels = Wheels()
        cmd_wheels.param[0] = self.right
        cmd_wheels.param[1] = self.left
        
        self.cmd_wheels_pub.publish(cmd_wheels)

    #############################################################
    def cmdVelCallback(self, msg):
    #############################################################
        self.cmd_dx = msg.linear.x
        self.cmd_dr = msg.angular.z

        self.update()

    #############################################################
    def parametersCallback(self):
    #############################################################
        self.radius = float(self.get_parameter('wheels.radius', 0.012).value) # The wheel radius in meters
        self.base_width = float(self.get_parameter('wheels.base_width', 0.245).value) # The wheel base width in meters
        


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
