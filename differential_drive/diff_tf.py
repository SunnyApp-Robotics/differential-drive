#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
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

from math import sin, cos, pi

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from tf2_ros import TransformBroadcaster

#############################################################################
class DiffTf(Node):
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        super().__init__("diff_tf")

        self.nodename = self.get_name()
        self.get_logger().info("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.rate = self.declare_parameter('rate',10.0).value  # the rate at which to publish the transform
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 50).value) # The number of wheel encoder ticks per meter of travel
        self.base_width = float(self.declare_parameter('base_width', 0.245).value) # The wheel base width in meters
        
        self.base_frame_id = self.declare_parameter('base_frame_id','base_link').value # the name of the base frame of the robot
        self.odom_frame_id = self.declare_parameter('odom_frame_id', 'odom').value # the name of the odometry reference frame
        
        self.encoder_min = self.declare_parameter('encoder_min', -32768).value
        self.encoder_max = self.declare_parameter('encoder_max', 32768).value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
 
        self.t_delta = Duration(seconds = 1.0/self.rate)
        self.t_next = self.get_clock().now() + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0.0               # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0.0
        self.rmult = 0.0
        self.prev_lencoder = 0.0
        self.prev_rencoder = 0.0
        self.x = 0.0                  # position in xy plane 
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0                 # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()
        
        # subscriptions
        self.create_subscription(Int16, "lwheel", self.lwheelCallback, qos_profile_system_default)
        self.create_subscription(Int16, "rwheel", self.rwheelCallback, qos_profile_system_default)
        self.odomPub = self.create_publisher(Odometry, "odom", qos_profile_system_default)

        # TF2 Broadcaster
        self.odomBroadcaster = TransformBroadcaster(self)

        duration = 1.0 / self.rate
        self.create_timer(duration, self.update)

    '''    
    #############################################################################
    def spin(self):
    #############################################################################
        r = rclpy.Rate(self.rate)
        while not rclpy.is_shutdown():
            self.update()
            r.sleep()
    ''' 
     
    #############################################################################
    def update(self):
    #############################################################################
        now = self.get_clock().now()
        if now > self.t_next:
            elapsed = now.seconds_nanoseconds()[1] - self.then.seconds_nanoseconds()[1]
            self.then = now
            elapsed = elapsed / 1000
            
            # calculate odometry
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (self.left - self.enc_left) / self.ticks_meter
                d_right = (self.right - self.enc_right) / self.ticks_meter
            self.enc_left = self.left
            self.enc_right = self.right
           
            # distance traveled is the average of the two wheels 
            d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = d / elapsed
            self.dr = th / elapsed
           
             
            if (d != 0):
                # calculate distance traveled in x and y
                x = cos( th ) * d
                y = -sin( th ) * d
                # calculate the final position of the robot
                self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
            if( th != 0):
                self.th = self.th + th
                
            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin( self.th / 2 )
            quaternion.w = cos( self.th / 2 )

            tfOdometry = TransformStamped()
            tfOdometry.header.frame_id = self.odom_frame_id
            tfOdometry.header.stamp = now.to_msg()
            tfOdometry.child_frame_id = self.base_frame_id
            tfOdometry.transform.translation.x = self.x
            tfOdometry.transform.translation.y = self.y
            tfOdometry.transform.translation.z = 0.0
            tfOdometry.transform.rotation.x = quaternion.x
            tfOdometry.transform.rotation.y = quaternion.y
            tfOdometry.transform.rotation.z = quaternion.z
            tfOdometry.transform.rotation.w = quaternion.w

            self.odomBroadcaster.sendTransform(tfOdometry)
            
            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)
            
            
    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

def main(args=None):
##########################################################################
##########################################################################
    rclpy.init(args=args)

    node = DiffTf()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown(0)


#############################################################################
#############################################################################
if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException: pass