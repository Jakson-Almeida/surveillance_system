#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

class IPCameraNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ip_camera_node', anonymous=True)
        
        # Initialize CV bridge for converting between OpenCV and ROS images
        self.bridge = CvBridge()
        
        # Get parameters from ROS parameter server
        self.rtsp_url = rospy.get_param('~rtsp_url', 'rtsp://admin:admin123@192.168.0.111:554/play1.sdp')
        self.frame_rate = rospy.get_param('~frame_rate', 30.0)
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/camera/camera_info')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        
        # Initialize publishers
        self.image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)
        self.camera_info_pub = rospy.Publisher(self.camera_info_topic, CameraInfo, queue_size=10)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.rtsp_url)
        
        if not self.cap.isOpened():
            rospy.logerr("Failed to open RTSP stream: %s", self.rtsp_url)
            rospy.signal_shutdown("Camera not accessible")
            return
        
        rospy.loginfo("IP Camera node initialized successfully")
        rospy.loginfo("Publishing to topic: %s", self.image_topic)
        
        # Set up camera info message
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = self.camera_frame
        self.camera_info.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.camera_info.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.camera_info.distortion_model = "plumb_bob"
        
        # Set rate for publishing
        self.rate = rospy.Rate(self.frame_rate)
        
    def run(self):
        """Main loop to capture and publish frames"""
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            
            if not ret:
                rospy.logwarn("Failed to read frame from camera")
                continue
            
            try:
                # Convert OpenCV image to ROS message
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = self.camera_frame
                
                # Publish image
                self.image_pub.publish(ros_image)
                
                # Publish camera info
                self.camera_info.header.stamp = ros_image.header.stamp
                self.camera_info_pub.publish(self.camera_info)
                
            except Exception as e:
                rospy.logerr("Error publishing image: %s", str(e))
            
            self.rate.sleep()
    
    def cleanup(self):
        """Clean up resources"""
        if self.cap.isOpened():
            self.cap.release()
        rospy.loginfo("IP Camera node shutdown")

if __name__ == '__main__':
    try:
        node = IPCameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'node' in locals():
            node.cleanup() 