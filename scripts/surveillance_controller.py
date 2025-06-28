#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose

class SurveillanceController:
    def __init__(self):
        rospy.init_node('surveillance_controller', anonymous=True)
        
        # Initialize parameters
        self.robot_speed = rospy.get_param('~robot_speed', 0.5)
        self.patrol_radius = rospy.get_param('~patrol_radius', 10.0)
        self.detection_threshold = rospy.get_param('~detection_threshold', 0.7)
        self.alert_cooldown = rospy.get_param('~alert_cooldown', 5.0)
        
        # Initialize state variables
        self.current_pose = None
        self.last_alert_time = rospy.Time.now()
        self.surveillance_mode = rospy.get_param('/surveillance_system/mode', 'autonomous')
        self.patrol_enabled = rospy.get_param('/surveillance_system/patrol_enabled', True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publishers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.alert_pub = rospy.Publisher('/surveillance/alerts', String, queue_size=10)
        self.status_pub = rospy.Publisher('/surveillance/status', String, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/processed_image', Image, self.image_callback)
        rospy.Subscriber('/surveillance/commands', String, self.command_callback)
        
        # Initialize patrol points
        self.patrol_points = self.generate_patrol_points()
        self.current_patrol_index = 0
        
        rospy.loginfo("Surveillance Controller initialized")
        
    def generate_patrol_points(self):
        """Generate patrol points in a circular pattern around the robot's initial position"""
        points = []
        num_points = 8
        for i in range(num_points):
            angle = 2 * np.pi * i / num_points
            x = self.patrol_radius * np.cos(angle)
            y = self.patrol_radius * np.sin(angle)
            points.append((x, y))
        return points
    
    def odom_callback(self, msg):
        """Callback for robot odometry"""
        self.current_pose = msg.pose.pose
        
    def scan_callback(self, msg):
        """Callback for laser scan data"""
        # Check for obstacles
        min_distance = min(msg.ranges)
        if min_distance < 0.5:  # Obstacle detected within 0.5m
            self.stop_robot()
            rospy.logwarn(f"Obstacle detected at distance: {min_distance}m")
    
    def image_callback(self, msg):
        """Callback for processed image data"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Check for detections in the image
            detections = self.analyze_image(cv_image)
            
            if detections:
                self.handle_detections(detections)
                
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")
    
    def analyze_image(self, cv_image):
        """Analyze image for potential threats or interesting objects"""
        detections = []
        
        # Convert to grayscale for processing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Simple motion detection (placeholder for more sophisticated detection)
        # In a real implementation, you would use more advanced computer vision techniques
        
        # Edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # Count edge pixels as a simple motion indicator
        edge_count = np.sum(edges > 0)
        
        if edge_count > 10000:  # Threshold for significant motion
            detections.append({
                'type': 'motion',
                'confidence': min(edge_count / 20000, 1.0),
                'location': 'center'
            })
        
        return detections
    
    def handle_detections(self, detections):
        """Handle detected objects or events"""
        current_time = rospy.Time.now()
        
        for detection in detections:
            if detection['confidence'] > self.detection_threshold:
                # Check if enough time has passed since last alert
                if (current_time - self.last_alert_time).to_sec() > self.alert_cooldown:
                    self.send_alert(detection)
                    self.last_alert_time = current_time
                    
                    # Stop robot and focus on detection
                    self.stop_robot()
                    self.focus_on_detection(detection)
    
    def send_alert(self, detection):
        """Send alert about detection"""
        alert_msg = f"Detection: {detection['type']} with confidence {detection['confidence']:.2f}"
        self.alert_pub.publish(alert_msg)
        rospy.logwarn(f"ALERT: {alert_msg}")
    
    def focus_on_detection(self, detection):
        """Focus camera and robot on detected object"""
        rospy.loginfo(f"Focusing on detection: {detection['type']}")
        # In a real implementation, you would control camera pan/tilt
        # and robot orientation to focus on the detection
    
    def command_callback(self, msg):
        """Handle surveillance commands"""
        command = msg.data.lower()
        
        if command == 'start_patrol':
            self.start_patrol()
        elif command == 'stop_patrol':
            self.stop_patrol()
        elif command == 'emergency_stop':
            self.emergency_stop()
        elif command == 'return_home':
            self.return_home()
        else:
            rospy.logwarn(f"Unknown command: {command}")
    
    def start_patrol(self):
        """Start autonomous patrol"""
        if self.patrol_enabled:
            rospy.loginfo("Starting autonomous patrol")
            self.patrol_enabled = True
            self.status_pub.publish("Patrol started")
    
    def stop_patrol(self):
        """Stop autonomous patrol"""
        rospy.loginfo("Stopping autonomous patrol")
        self.patrol_enabled = False
        self.stop_robot()
        self.status_pub.publish("Patrol stopped")
    
    def emergency_stop(self):
        """Emergency stop all robot movement"""
        rospy.logwarn("EMERGENCY STOP ACTIVATED")
        self.stop_robot()
        self.patrol_enabled = False
        self.status_pub.publish("Emergency stop")
    
    def return_home(self):
        """Return robot to home position"""
        rospy.loginfo("Returning to home position")
        home_pose = PoseStamped()
        home_pose.header.frame_id = "map"
        home_pose.header.stamp = rospy.Time.now()
        home_pose.pose.position.x = 0.0
        home_pose.pose.position.y = 0.0
        home_pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(home_pose)
        self.status_pub.publish("Returning home")
    
    def stop_robot(self):
        """Stop robot movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def move_to_patrol_point(self, point):
        """Move robot to a specific patrol point"""
        if self.current_pose is None:
            return
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal_pose)
        rospy.loginfo(f"Moving to patrol point: ({point[0]:.2f}, {point[1]:.2f})")
    
    def patrol_loop(self):
        """Main patrol loop"""
        if not self.patrol_enabled:
            return
        
        # Move to next patrol point
        if self.current_pose is not None:
            current_point = self.patrol_points[self.current_patrol_index]
            self.move_to_patrol_point(current_point)
            
            # Move to next point after some time
            rospy.sleep(10.0)  # Wait 10 seconds at each point
            self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
    
    def run(self):
        """Main run loop"""
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            try:
                # Execute patrol if enabled
                if self.patrol_enabled:
                    self.patrol_loop()
                
                # Publish status
                status = f"Mode: {self.surveillance_mode}, Patrol: {self.patrol_enabled}"
                self.status_pub.publish(status)
                
                rate.sleep()
                
            except rospy.ROSInterruptException:
                rospy.loginfo("Surveillance Controller interrupted")
                break
            except Exception as e:
                rospy.logerr(f"Error in surveillance controller: {e}")

if __name__ == '__main__':
    try:
        controller = SurveillanceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass 