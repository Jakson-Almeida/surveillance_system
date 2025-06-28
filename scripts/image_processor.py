#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import time

class ImageProcessor:
    def __init__(self):
        rospy.init_node('image_processor', anonymous=True)
        
        # Initialize parameters
        self.input_topic = rospy.get_param('~input_topic', '/camera/image_raw')
        self.output_topic = rospy.get_param('~output_topic', '/processed_image')
        self.detection_enabled = rospy.get_param('~detection_enabled', True)
        self.motion_detection = rospy.get_param('~motion_detection', True)
        self.face_detection = rospy.get_param('~face_detection', True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize state variables
        self.previous_frame = None
        self.motion_threshold = 5000
        self.face_cascade = None
        
        # Load face detection cascade if enabled
        if self.face_detection:
            try:
                self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
                if self.face_cascade.empty():
                    rospy.logwarn("Could not load face detection cascade")
                    self.face_detection = False
                else:
                    rospy.loginfo("Face detection cascade loaded successfully")
            except Exception as e:
                rospy.logwarn(f"Error loading face detection: {e}")
                self.face_detection = False
        
        # Publishers
        self.processed_image_pub = rospy.Publisher(self.output_topic, Image, queue_size=10)
        self.detection_pub = rospy.Publisher('/surveillance/detections', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber(self.input_topic, Image, self.image_callback)
        rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        
        rospy.loginfo("Image Processor initialized")
        
    def camera_info_callback(self, msg):
        """Callback for camera info"""
        # Store camera parameters if needed
        pass
        
    def image_callback(self, msg):
        """Main image processing callback"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            processed_image = self.process_image(cv_image)
            
            # Publish processed image
            if processed_image is not None:
                ros_image = self.bridge.cv2_to_imgmsg(processed_image, "bgr8")
                ros_image.header = msg.header
                self.processed_image_pub.publish(ros_image)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def process_image(self, image):
        """Main image processing function"""
        if image is None:
            return None
            
        # Create a copy for processing
        processed = image.copy()
        
        # Apply image enhancements
        processed = self.enhance_image(processed)
        
        # Perform motion detection
        if self.motion_detection and self.detection_enabled:
            motion_detected = self.detect_motion(processed)
            if motion_detected:
                rospy.loginfo("Motion detected in image")
                self.detection_pub.publish("motion_detected")
        
        # Perform face detection
        if self.face_detection and self.detection_enabled:
            faces = self.detect_faces(processed)
            if faces:
                rospy.loginfo(f"Faces detected: {len(faces)}")
                self.detection_pub.publish(f"faces_detected:{len(faces)}")
        
        # Add processing information overlay
        processed = self.add_overlay(processed)
        
        return processed
    
    def enhance_image(self, image):
        """Enhance image quality for better detection"""
        # Convert to LAB color space for better processing
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        lab[:,:,0] = clahe.apply(lab[:,:,0])
        
        # Convert back to BGR
        enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        # Apply slight Gaussian blur to reduce noise
        enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
        
        return enhanced
    
    def detect_motion(self, current_frame):
        """Detect motion between current and previous frame"""
        if self.previous_frame is None:
            self.previous_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
            return False
        
        # Convert current frame to grayscale
        current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        
        # Calculate absolute difference
        frame_diff = cv2.absdiff(self.previous_frame, current_gray)
        
        # Apply threshold to get binary image
        _, thresh = cv2.threshold(frame_diff, 25, 255, cv2.THRESH_BINARY)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5,5), np.uint8)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # Count non-zero pixels
        motion_pixels = cv2.countNonZero(thresh)
        
        # Update previous frame
        self.previous_frame = current_gray
        
        # Return True if motion is detected
        return motion_pixels > self.motion_threshold
    
    def detect_faces(self, image):
        """Detect faces in the image"""
        if self.face_cascade is None:
            return []
        
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        # Draw rectangles around detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(image, 'Face', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        
        return faces
    
    def add_overlay(self, image):
        """Add information overlay to the image"""
        # Add timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(image, timestamp, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Add processing status
        status = f"Motion: {'ON' if self.motion_detection else 'OFF'}, Face: {'ON' if self.face_detection else 'OFF'}"
        cv2.putText(image, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add frame counter or other info
        cv2.putText(image, "Surveillance Active", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return image
    
    def run(self):
        """Main run loop"""
        rospy.loginfo("Image Processor running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        processor = ImageProcessor()
        processor.run()
    except rospy.ROSInterruptException:
        pass 