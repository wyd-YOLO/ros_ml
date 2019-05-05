#!/usr/bin/env python2

"""!@file yolo_detection_node.py
@author Nguyen Quang <nguyenquang.emailbox@gmail.com>
@brief The YOLO detection node class.
@since 0.0.1

@copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.

"""

import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from darkflow.net.build import TFNet


class YOLODetection:
    """!@brief A class to subscribe to an image topic and detect the desired objects in the image using YOLO.
    @since 0.0.1
    """

    def __init__(self):
        """!@brief Construct a new YOLODetection object.
        @since 0.0.1
        """

        # Define the model options and run
        TFNET_OPTIONS = {
            "model": "cfg/tiny-yolo-voc-2c.cfg",
            "load": 10000,
            "threshold": 0.1,
            "gpu": 0.8
        }
        self.__tfnet = TFNet(TFNET_OPTIONS)

        # The image publisher
        self.__image_pub = rospy.Publisher("/yolo/compressed", CompressedImage, queue_size=0)

        # The image subscriber
        self.__image_sub = rospy.Subscriber("/econ_camera/image_raw/compressed", CompressedImage, self.image_callback)

    def image_callback(self, image_msg):
        """!@brief The image callback function.
        @since 0.0.1
        """

        np_arr = np.fromstring(image_msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image = cv2.flip(image, -1)

        # Use YOLO to detect objects in the image
        results = self.__tfnet.return_predict(image)
        for result in results:
            tl = (result["topleft"]["x"], result["topleft"]["y"])
            br = (result["bottomright"]["x"], result["bottomright"]["y"])
            confidence = str(result["confidence"])[:4]
            label = result["label"]

            # Add the box and confidence
            if label == "giant_location":
                image = cv2.rectangle(image, tl, br, (255, 0, 0), 2)
                image = cv2.putText(image, confidence, (tl[0], tl[1] + 25), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
            elif label == "qr_tag":
                image = cv2.rectangle(image, tl, br, (0, 0, 255), 2)
                image = cv2.putText(image, confidence, (tl[0], tl[1] + 25), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

        # Compress and publish the result image
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        self.__image_pub.publish(msg)


def main(args):
    """!@brief The main function.
    @since 0.0.1
    """

    ic = YOLODetection()
    rospy.init_node("yolo_detection", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
