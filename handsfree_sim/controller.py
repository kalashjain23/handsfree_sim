#!/usr/bin/env python3

import cv2 as cv
import rclpy
from rclpy.node import Node
from handsfree_sim.brain import movement
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class Controller(Node):
    
    def __init__(self):
        super().__init__('controller')
        
        self.cap_ = cv.VideoCapture(0)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.controller_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.image_publisher_ = self.create_publisher(Image, "frames", 10)
        self.image_subscriber_ = self.create_subscription(Image, "frames", self.show_image, 10)
        
        self.velocity_ = Twist()
        self.br = CvBridge()
        
    def timer_callback(self):
        success, image = self.cap_.read()
        
        if success:      
            match movement(image):
                case "left":
                    self.velocity_.linear.x = 1.5
                    self.velocity_.angular.z = 1.5
                case 'sleft':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = 1.5
                case 'right':
                    self.velocity_.linear.x = 1.5
                    self.velocity_.angular.z = -1.5
                case 'sright':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = -1.5
                case 'straight':
                    self.velocity_.linear.x = 1.5
                    self.velocity_.angular.z = 0.0
                case 'stop':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = 0.0
                
            self.controller_.publish(self.velocity_)
            image = cv.flip(image, 1)
            self.image_publisher_.publish(self.br.cv2_to_imgmsg(image))
            
    def show_image(self, data):
        frame = self.br.imgmsg_to_cv2(data)
        cv.imshow('Controller', frame)
        cv.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    