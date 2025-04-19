#!/usr/bin/env python3

import rclpy

def main():
  rclpy.init()
  node = rclpy.create_node( 'my_first_node' )
  rclpy.spin( node )

if __name__ == '__main__':
  main()

