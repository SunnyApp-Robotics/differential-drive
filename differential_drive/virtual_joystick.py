#!/usr/bin/env python

"""

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
import sys
import inspect, os
import threading

#import roslib; roslib.load_manifest('differential_drive')
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

from geometry_msgs.msg import Twist

from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow

##########################################################################
##########################################################################
class VirtualJoystick(Node):

    def __init__(self):

        super().__init__('virtual_joystick')

        ### Parameter declaraiton
        self.x_min_p = self.declare_parameter("x_min", -0.20)
        self.x_max_p = self.declare_parameter("x_max", 0.20)
        self.r_min_p = self.declare_parameter("r_min", -1.0)
        self.r_max_p = self.declare_parameter("r_max", 1.0)

        self.publish_rate_p = self.declare_parameter("publish_rate", 50)

        self.get_logger().info('virtual_joystick started')
        

##########################################################################
##########################################################################
class MainWindow(QMainWindow):
##########################################################################
##########################################################################

    #####################################################################    
    def __init__(self, node):
    #####################################################################    
        super(MainWindow, self).__init__()
        self.node = node
        self.timer_rate = self.node.publish_rate_p.value
        self.pub_twist = self.node.create_publisher(Twist, 'cmd_vel', qos_profile_system_default)
        
        self.initUI()
        
    #####################################################################    
    def initUI(self):      
    #####################################################################    
        
        img_path = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) + "./../images/crosshair.jpg"
        self.node.get_logger().info('initUI img_path: %s' % img_path)

        self.statusBar()
        
        self.setStyleSheet("QMainWindow { border-image: url(%s); }" % img_path)
        
                
        self.setGeometry(0, 600, 200, 200)
        self.setWindowTitle('Virtual Joystick')
        self.show()
        self.timer = QtCore.QBasicTimer()
        
        self.statusBar().showMessage('started')
        
    #####################################################################    
    def mousePressEvent(self, event):
    #####################################################################    
        self.statusBar().showMessage('mouse clicked')
        self.timer.start(self.timer_rate, self)
        self.get_position(event)
        
    #####################################################################    
    def mouseReleaseEvent(self, event):
    #####################################################################    
        self.statusBar().showMessage('mouse released')
        self.timer.stop()
        
    #####################################################################    
    def mouseMoveEvent(self, event):
    #####################################################################    
        self.get_position(event)
        
    #####################################################################    
    def get_position(self, event):
    #####################################################################    
        s = self.size()
        s_w = s.width()
        s_h = s.height()
        pos = event.pos()
        self.x = 1.0 * pos.x() / s_w
        self.y = 1.0 * pos.y() / s_h
        
        self.statusBar().showMessage('point (%0.2f, %0.2f)' % (self.x,self.y))
        
    #####################################################################    
    def timerEvent(self, event):
    #####################################################################    
        # self.statusBar().showMessage("timer tick")
        self.pubTwist()
        
    #######################################################
    def pubTwist(self):
    #######################################################
        # rclpy.get_logger().info("publishing twist from (%0.3f,%0.3f)" %(self.x,self.y))
        x_max = self.node.x_max_p.value 
        x_min = self.node.x_min_p.value 
        r_max = self.node.r_max_p.value 
        r_min = self.node.r_min_p.value 

        self.twist = Twist()
        self.twist.linear.x = (1.0-self.y) * (x_max - x_min) + x_min
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = (1.0-self.x) * (r_max - r_min) + r_min
        
        if self.twist.linear.x > x_max:
            self.twist.linear.x = x_max
        if self.twist.linear.x < x_min:
            self.twist.linear.x = x_min
        if self.twist.angular.z > r_max:
            self.twist.angular.z = r_max
        if self.twist.angular.z < r_min:
            self.twist.angular.z = r_min
        
        self.pub_twist.publish( self.twist )

def execNode(node):
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shotdown(0)
        
##########################################################################
##########################################################################
def main(args=None):
##########################################################################
##########################################################################
    rclpy.init(args=args)

    node = VirtualJoystick()

    app = QApplication(sys.argv)

    ### Create MainWindow class inside node declaration
    threadQT = threading.Thread(target = execNode, args=[node])
    threadQT.daemon = True
    threadQT.start()

    ex = MainWindow(node)
    sys.exit(app.exec_())
    

if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException: pass
