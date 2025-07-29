#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class FaceDetectorNode:
    def __init__(self):
        rospy.init_node('face_detection_node')

        self.bridge = CvBridge()

        # MediaPipe face mesh (or use FaceDetection for bounding box only)
        self.mp_face = mp.solutions.face_mesh
        self.face_mesh = self.mp_face.FaceMesh(
            static_image_mode=False,
            max_num_faces=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)
        rospy.loginfo("Face detection node started. Waiting for images...")
        rospy.spin()

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        # Convert to RGB for MediaPipe
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process with MediaPipe
        results = self.face_mesh.process(rgb_image)

        # Draw face mesh
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                self.mp_drawing.draw_landmarks(
                    image=image,
                    landmark_list=face_landmarks,
                    connections=self.mp_face.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style()
                )

        # Show result
        cv2.imshow("MediaPipe Face Mesh", image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        FaceDetectorNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
