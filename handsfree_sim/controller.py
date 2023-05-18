#!/usr/bin/env python3

import cv2 as cv
import rclpy
from rclpy.node import Node
from handsfree_sim.brain import movement

from geometry_msgs.msg import Twist

class Controller(Node):
    
    def __init__(self):
        super().__init__('controller')
        
        self.cap_ = cv.VideoCapture(0)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.velocity_ = Twist()
        
    def timer_callback(self):
        success, image = self.cap_.read()
        
        if success:      
            match movement(image):
                case "l":
                    self.velocity_.linear.x = 1.5
                    self.velocity_.angular.z = 0.75
                case 'ls':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = 0.75
                case 'r':
                    self.velocity_.linear.x = 1.5
                    self.velocity_.angular.z = -0.75
                case 'rs':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = -0.75
                case 'n':
                    self.velocity_.linear.x = 0.0
                    self.velocity_.angular.z = 0.0
                
            self.publisher_.publish(self.velocity_)
            cv.imshow('controller', image)
        
        
def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    