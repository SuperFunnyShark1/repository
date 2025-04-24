import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
import numpy as np
from cv_bridge import CvBridge
import cv2




NODE_NAME:str = "obstacle_detector"



class Obstacle_detector(Node): 

    def __init__( self ):

        super().__init__(NODE_NAME)

        self.image_raw_subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10
        )
        
        self.occupancy_state_publisher = self.create_publisher(
            Vector3,
            '/occupancy_state',
            10
        )
        

        
        

        
        

        self.subscriptions  # prevent unused variable warning


    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        
        print("Image width: ", msg.width)
    







if __name__ == '__main__':
    rclpy.init()
  
    detector_node = Obstacle_detector()

  
    rclpy.spin( detector_node )
    
    detector_node.destroy_node()
    
    rclpy.shutdown()

