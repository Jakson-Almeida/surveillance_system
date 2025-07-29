#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class UsbCamViewer:
    def __init__(self):
        rospy.init_node('usb_cam_viewer', anonymous=True)

        self.bridge = CvBridge()

        rospy.Subscriber('/usb_cam/image_raw', Image, self.callback)

        rospy.loginfo("usb_cam_viewer started. Waiting for image frames...")
        rospy.spin()

    def callback(self, data):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge error: %s", e)
            return

        cv2.imshow("Webcam View (/usb_cam/image_raw)", cv_image)
        cv2.waitKey(1)  # Needed for imshow to refresh


if __name__ == '__main__':
    try:
        UsbCamViewer()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

