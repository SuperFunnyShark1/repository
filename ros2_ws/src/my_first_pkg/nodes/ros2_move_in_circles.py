#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveInCircles( Node ):

  def __init__( self ):
    super().__init__( 'move_in_circles_node' )
    self.max_v = 1.0 # [m/s]
    self.max_w = 2.0 # [rad/s]
    self.cmd_vel_mux_pub = self.create_publisher( Twist, '/cmd_vel_mux/input/navigation', 10 )
    timer_period = 0.2  # [s]
    self.timer = self.create_timer( timer_period, self.move )

  def move( self ):
    speed = Twist()
    speed.linear.x = self.max_v
    speed.angular.z = self.max_w
    self.get_logger().info( 'publishing speed (%f, %f)' % (self.max_v, self.max_w) )
    self.cmd_vel_mux_pub.publish( speed )


if __name__ == '__main__':
  rclpy.init()
  mic = MoveInCircles()
  rclpy.spin( mic )

