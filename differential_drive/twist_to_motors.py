#!/usr/bin/env python
"""
   twist_to_motors - converts a twist message to motor commands.  Needed for navigation stack
   
   
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
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

#############################################################
#############################################################


class TwistToMotors(Node):
    #############################################################
    #############################################################

    #############################################################
    def __init__(self):
        #############################################################
        super().__init__("twist_to_motors")
        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)

        self.w = self.declare_parameter("base_width", 0.2).value
        self.rate = self.declare_parameter("rate", 50).value
        self.timeout_ticks = self.declare_parameter("timeout_ticks", 2).value
        self.ticks_since_target = self.timeout_ticks
        self.left = 0
        self.right = 0

        self.pub_lmotor = self.create_publisher(
            Float32, 'lwheel_vtarget', qos_profile_system_default)
        self.pub_rmotor = self.create_publisher(
            Float32, 'rwheel_vtarget', qos_profile_system_default)
        self.create_subscription(Twist, 'cmd_vel', self.twistCallback, qos_profile_system_default)

        duration = 1 / self.rate
        self.create_timer(duration, self.update)

    #############################################################
    def update(self):
        #############################################################
        if self.ticks_since_target < self.timeout_ticks:

            # dx = (l + r) / 2
            # dr = (r - l) / w

            self.right = 1.0 * self.dx + self.dr * self.w / 2
            self.left = 1.0 * self.dx - self.dr * self.w / 2
            # rospy.loginfo("publishing: (%d, %d)", left, right)

            self.pub_lmotor.publish(self.left)
            self.pub_rmotor.publish(self.right)

            self.ticks_since_target += 1

    #############################################################
    def twistCallback(self, msg):
        #############################################################
        # self.get_logger().info("-D- twistCallback: %s" % str(msg))
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dy = msg.linear.y
        self.dr = msg.angular.z


def main(args=None):
    ##########################################################################
    ##########################################################################
    rclpy.init(args=args)

    node = TwistToMotors()
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
