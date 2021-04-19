#!/usr/bin/env python

"""
   Odometry (diff_tf) - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
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
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.+

"""

from math import sin, cos
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from common.msg import Wheels

#############################################################################


class Odometry_Encoders(Node):
    #############################################################################

    #############################################################################
    def __init__(self):
        #############################################################################
        super().__init__("odometry_encoders")

        self.nodename = self.get_name()
        self.get_logger().info("%s started" % self.nodename)

        #### parameters #######
        self.radius = float(self.declare_parameter(
            'wheels.radius', 0.02569).value)  # The wheel radius in meters
        self.base_width = float(self.declare_parameter(
            'wheels.base_width', 0.1275).value)  # The wheel base width in meters

        # the name of the base frame of the robot
        self.base_frame_id = self.declare_parameter(
            'base_frame_id', 'base_link').value
        # the name of the odometry reference frame
        self.odom_frame_id = self.declare_parameter(
            'odom_frame_id', 'odom').value

        self.ticks_mode = self.declare_parameter(
            'ticks.ticks_mode', False).value
        # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = float(self.declare_parameter(
            'ticks.ticks_meter', 50.0).value)
        self.encoder_min = self.declare_parameter(
            'ticks.encoder_min', -32768).value
        self.encoder_max = self.declare_parameter(
            'ticks.encoder_max', 32768).value

        # Init variables
        self.init()

        # subscriptions / publishers
        self.create_subscription(
            Wheels, "robot/enc_wheels", self.wheelsCallback, qos_profile_system_default)
        self.create_subscription(
            Wheels, "robot/enc_ticks_wheels", self.wheelsEncCallback, qos_profile_system_default)
        self.cal_vel_pub = self.create_publisher(
            Twist, "cal_vel", qos_profile_system_default)
        self.odom_pub = self.create_publisher(
            Odometry, "odom", qos_profile_system_default)

        # 5 seconds timer to update parameters
        self.create_timer(5, self.parametersCallback)

        # TF2 Broadcaster
        self.odomBroadcaster = TransformBroadcaster(self)

    #############################################################################
    def init(self):
    #############################################################################

        # Internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.lvel = 0.0
        self.rvel = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0.0
        self.prev_rencoder = 0.0
        self.encoder_low_wrap = (
            self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (
            self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min

        # Actual values coming back from robot
        self.left = 0.0           
        self.right = 0.0

        # Position in xy plane
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        # Linear and angular velocities
        self.linear_accumulator = {"sum": 0.0, "buffer": np.zeros(
            10), "next_insert": 0, "buffer_filled": False}
        self.angular_accumulator = {"sum": 0.0, "buffer": np.zeros(
            10), "next_insert": 0, "buffer_filled": False}
        self.dt_accumulator = {"sum": 0.0, "buffer": np.zeros(
            5), "next_insert": 0, "buffer_filled": False}

        self.then = self.get_clock().now()

    #############################################################################
    def update(self):
        #############################################################################
        now = self.get_clock().now()  # Current time

        # Elapsed time [nanoseconds]
        elapsed = now.nanoseconds - self.then.nanoseconds
        elapsed = float(elapsed) / 1e9  # Elapsed time [seconds]        
        self.then = now  # Update previous time
        self.dt_accumulator = self.accumulateMean(self.dt_accumulator, elapsed)

        self.calculateOdometry(self.getRollingMean(self.dt_accumulator))  # Calculate Odometry
        self.publishCalVel(self.getRollingMean(self.dt_accumulator))  # Publish Calculated Velocities
        self.publishOdometry(now)  # Publish Odometry

    #############################################################################
    def calculateOdometry(self, elapsed):
        #############################################################################

        # calculate odometry
        if self.ticks_mode:
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
        else:
            d_left = self.left * self.radius
            d_right = self.right * self.radius

        # Linear velocity
        d = (d_left + d_right) / 2
        # Angular velocity
        th = (d_right - d_left) / (2 * self.base_width)
        # calculate velocities
        dx = d / elapsed
        dr = th / elapsed

        self.linear_accumulator = self.accumulateMean(
            self.linear_accumulator, dx)
        self.angular_accumulator = self.accumulateMean(
            self.angular_accumulator, dr)

        # Accumulate

        # Calculate distance traveled in x and y
        x = cos(th) * d
        y = -sin(th) * d

        # Calculate the final position of the robot
        self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
        self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        self.th = self.th + th

    #############################################################################
    def accumulateMean(self, dict, val):
        #############################################################################
        dict["sum"] = dict["sum"] - dict["buffer"][dict["next_insert"]]
        dict["sum"] = dict["sum"] + val
        dict["buffer"][dict["next_insert"]] = val
        dict["next_insert"] = dict["next_insert"] + 1
        dict["buffer_filled"] = dict["buffer_filled"] or (
            dict["next_insert"] >= len(dict["buffer"]))
        dict["next_insert"] = dict["next_insert"] % len(dict["buffer"])
        return dict

    #############################################################################
    def getRollingMean(self, dict):
        #############################################################################
        valid_data_count = dict["buffer_filled"] * len(dict["buffer"]) + (
            not dict["buffer_filled"]) * dict["next_insert"]
        return float(dict["sum"] / valid_data_count)

    #############################################################################
    def publishOdometry(self, now):
        #############################################################################

        # publish the odom information
        quaternion = Quaternion()

        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)

        # TF Broadcaster

        tfOdometry = TransformStamped()
        tfOdometry.header.stamp = now.to_msg()
        tfOdometry.header.frame_id = self.odom_frame_id
        tfOdometry.child_frame_id = self.base_frame_id

        tfOdometry.transform.translation.x = self.x
        tfOdometry.transform.translation.y = self.y
        tfOdometry.transform.translation.z = 0.0

        tfOdometry.transform.rotation.x = quaternion.x
        tfOdometry.transform.rotation.y = quaternion.y
        tfOdometry.transform.rotation.z = quaternion.z
        tfOdometry.transform.rotation.w = quaternion.w

        self.odomBroadcaster.sendTransform(tfOdometry)

        # Odometry

        odom = Odometry()
        odom.header.stamp = now.to_msg()

        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion

        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.getRollingMean(
            self.linear_accumulator)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.getRollingMean(
            self.angular_accumulator)

        self.odom_pub.publish(odom)

    #############################################################################
    def publishCalVel(self, elapsed):
        #############################################################################

        cal_vel = Twist()
        cal_vel.linear.x = self.getRollingMean(self.linear_accumulator)
        cal_vel.angular.z = self.getRollingMean(self.angular_accumulator)
        cal_vel.linear.z = elapsed

        self.cal_vel_pub.publish(cal_vel)

    #############################################################################
    def wheelsEncCallback(self, msg):
        #############################################################################

        # Right Wheel Encoder
        encRight = msg.param[0]

        if(encRight < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1

        if(encRight > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1

        self.right = 1.0 * (encRight + self.rmult *
                            (self.encoder_max - self.encoder_min))
        self.prev_rencoder = encRight

        # Left Wheel Encoder
        encLeft = msg.param[1]

        if (encLeft < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1

        if (encLeft > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1

        self.left = 1.0 * (encLeft + self.lmult *
                           (self.encoder_max - self.encoder_min))
        self.prev_lencoder = encLeft

        if self.ticks_mode:
            self.update()

    #############################################################################
    def wheelsCallback(self, msg):
        #############################################################################
        self.right = msg.param[0]
        self.left = msg.param[1]

        if not self.ticks_mode:
            self.update()

    #############################################################################
    def parametersCallback(self):
        #############################################################################
        # The wheel radius in meters
        self.radius = float(self.get_parameter('wheels.radius').value)
        # The wheel base width in meters
        self.base_width = float(self.get_parameter(
            'wheels.base_width').value)

        # the name of the base frame of the robot
        self.base_frame_id = self.get_parameter('base_frame_id').value
        # the name of the odometry reference frame
        self.odom_frame_id = self.get_parameter('odom_frame_id').value

        self.ticks_mode = self.get_parameter('ticks.ticks_mode').value
        # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = float(self.get_parameter('ticks.ticks_meter').value)
        self.encoder_min = self.get_parameter('ticks.encoder_min').value
        self.encoder_max = self.get_parameter('ticks.encoder_max').value

        self.encoder_low_wrap = (
            self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (
            self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min


def main(args=None):
    ##########################################################################
    ##########################################################################
    rclpy.init(args=args)

    node = Odometry_Encoders()
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
