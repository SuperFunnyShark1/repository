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
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.occupancy_state_publisher = self.create_publisher(
            Vector3,
            '/occupancy_state',
            10
        )
        
        self.bridge = CvBridge()

        
        
                
        
        

        self.subscriptions  # prevent unused variable warning


    def image_callback(self, msg):
        self.get_logger().info('Received image')
        
        
        print("Image width: ", msg.width)
    

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Display the image
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)






if __name__ == '__main__':
    rclpy.init()
  
    detector_node = Obstacle_detector()

  
    rclpy.spin( detector_node )
    
    detector_node.destroy_node()
    
    rclpy.shutdown()

