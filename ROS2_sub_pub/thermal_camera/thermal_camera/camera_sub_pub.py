#!/usr/bin/env python3

import random
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Float64MultiArray


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')
        
        self.rgb_publisher_ = self.create_publisher(Image, 'teraranger_evo_thermal/rgb_image', 100)
        self.raw_publisher_ = self.create_publisher(Float64MultiArray, 'teraranger_evo_thermal/raw_temp_array', 100)
        self.ptat_publisher_ = self.create_publisher(Float64, 'teraranger_evo_thermal/ptat', 100)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def rgb_publish(self, rgb):
        pub = Image()
        bridge = CvBridge()
        pub = rgb #bridge.cv2_to_imgmsg
        # self.get_logger().info(f'Publishing: {camera}')
        self.rgb_publisher_.publish(pub)
    
    def raw_publish(self, raw):
        pub = Float64MultiArray()
        pub = raw
        # self.get_logger().info(f'Publishing: {camera}')
        self.raw_publisher_.publish(pub)

    def ptat_publish(self, ptat):
        pub = Float64()
        pub = ptat
        # self.get_logger().info(f'Publishing: {camera}')
        self.ptat_publisher_.publish(pub)        
        
    def timer_callback(self):
        quit()     


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')
        self.camera_pub = CameraPublisher()

        self.rgb_subscription = self.create_subscription(Image, 'teraranger_evo_thermal/rgb_image', self.rgb_callback, 100)
        self.raw_subscription = self.create_subscription(Float64MultiArray, '/teraranger_evo_thermal/raw_temp_array', self.raw_callback, 100)
        self.ptat_subscription = self.create_subscription(Float64, 'teraranger_evo_thermal/ptat', self.ptat_callback, 100)

        self.rgb_subscription  # prevent unused variable warning
        self.raw_subscription  # prevent unused variable warning
        self.ptat_subscription  # prevent unused variable warning

    def rgb_callback(self, rgb):
        # print ("I heard: ", rgb)
        # Publish 
        self.camera_pub.rgb_publish(rgb)

    def raw_callback(self, raw):
        # print ("I heard: ", raw)
        # Publish 
        self.camera_pub.raw_publish(raw)

    def ptat_callback(self, ptat):
        # print ("I heard: ", ptat)
        # Publish 
        self.camera_pub.ptat_publish(ptat)



def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    # camera_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()