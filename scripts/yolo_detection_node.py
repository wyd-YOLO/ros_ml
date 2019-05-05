#!/usr/bin/env python2
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class YoloDetection:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/econ_camera/image_raw/compressed", CompressedImage, self.callback)

    def callback(self, image_msg):
        try:
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv2.imshow("cv_image", cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


def main():
    ic = YoloDetection()
    rospy.init_node("YoloDetection", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
