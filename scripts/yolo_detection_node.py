#!/usr/bin/env python2

"""!@file yolo_detection_node.py
@author Nguyen Quang <nguyenquang.emailbox@gmail.com>
@brief The YOLO detection node class.
@since 0.0.1

@copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.

"""

import sys
import rospy
import std_msgs.msg
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from darkflow.net.build import TFNet
from ros_ml.msg import Pixel, YoloObject, YoloResult


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
            "load": 9500,
            "threshold": 0.1,
            "gpu": 0.8
        }
        self.__tfnet = TFNet(TFNET_OPTIONS)

        # Check whether the input image topic is compressed
        self.__yolo_compressed_topic = rospy.get_param("~yolo_compressed_topic", True)

        # The YOLO detection input image subscriber
        yolo_input_image_topic = rospy.get_param("~yolo_input_image_topic", "/econ_camera/image_raw/compressed")
        self.__yolo_image_sub = None
        if self.__yolo_compressed_topic == True:
            self.__yolo_image_sub = rospy.Subscriber(yolo_input_image_topic, CompressedImage, self.image_callback)
        else:
            self.__yolo_image_sub = rospy.Subscriber(yolo_input_image_topic, Image, self.image_callback)

        # The camera rotation in [0, 1, 2, 3] <-> [0, 90, 180, 270]
        self.__rotation = rospy.get_param("~rotation", 0)

        # The cv_bridge
        self.__cv_bridge = CvBridge()

        # The frame index
        self.__frame_idx = 0

        # The YOLO detection image publisher
        yolo_image_mod_0_topic = rospy.get_param("~yolo_image_mod_0_topic", "yolo_detection_mod_0_image")
        self.__yolo_image_mod_0_pub = rospy.Publisher(yolo_image_mod_0_topic, Image, queue_size=0)
        yolo_image_mod_1_topic = rospy.get_param("~yolo_image_mod_1_topic", "yolo_detection_mod_1_image")
        self.__yolo_image_mod_1_pub = rospy.Publisher(yolo_image_mod_1_topic, Image, queue_size=0)
        yolo_image_mod_2_topic = rospy.get_param("~yolo_image_mod_2_topic", "yolo_detection_mod_2_image")
        self.__yolo_image_mod_2_pub = rospy.Publisher(yolo_image_mod_2_topic, Image, queue_size=0)
        yolo_image_mod_3_topic = rospy.get_param("~yolo_image_mod_3_topic", "yolo_detection_mod_3_image")
        self.__yolo_image_mod_3_pub = rospy.Publisher(yolo_image_mod_3_topic, Image, queue_size=0)

        # The YOLO detection result publisher
        yolo_result_mod_0_topic = rospy.get_param("~yolo_result_mod_0_topic", "yolo_detection_mod_0_result")
        self.__yolo_result_mod_0_pub = rospy.Publisher(yolo_result_mod_0_topic, YoloResult, queue_size=0)
        yolo_result_mod_1_topic = rospy.get_param("~yolo_result_mod_1_topic", "yolo_detection_mod_1_result")
        self.__yolo_result_mod_1_pub = rospy.Publisher(yolo_result_mod_1_topic, YoloResult, queue_size=0)
        yolo_result_mod_2_topic = rospy.get_param("~yolo_result_mod_2_topic", "yolo_detection_mod_2_result")
        self.__yolo_result_mod_2_pub = rospy.Publisher(yolo_result_mod_2_topic, YoloResult, queue_size=0)
        yolo_result_mod_3_topic = rospy.get_param("~yolo_result_mod_3_topic", "yolo_detection_mod_3_result")
        self.__yolo_result_mod_3_pub = rospy.Publisher(yolo_result_mod_3_topic, YoloResult, queue_size=0)

    def image_callback(self, image_msg):
        """!@brief The image callback function.
        @since 0.0.1
        """

        # Decode the image
        image = None
        if self.__yolo_compressed_topic == True:
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            try:
                image = self.__cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print(e)

        # Rotate the image
        if self.__rotation != 0:
            if self.__rotation == 1:
                image = cv2.transpose(image)
                image = cv2.flip(image, 1)
            elif self.__rotation == 2:
                image = cv2.flip(image, -1)
            elif self.__rotation == 3:
                image = cv2.transpose(image)
                image = cv2.flip(image, 0)

        qr_tags = []
        giant_locations = []
        # Use YOLO to detect objects in the image
        results = self.__tfnet.return_predict(image)
        for result in results:
            tl = Pixel(x=result["topleft"]["x"], y=result["topleft"]["y"])
            br = Pixel(x=result["bottomright"]["x"], y=result["bottomright"]["y"])
            confidence = result["confidence"]
            rect = YoloObject(tl=tl, br=br, confidence=confidence)
            label = result["label"]

            # Add the box and confidence
            if label == "giant_location":
                rect.br.x = rect.tl.x + int((rect.br.x - rect.tl.x) * 0.75)
                shift_left = 5
                if rect.tl.x - shift_left > 0:
                    rect.tl.x = rect.tl.x - shift_left
                else:
                    rect.tl.x = 0
                if rect.br.x - shift_left > 0:
                    rect.br.x = rect.br.x - shift_left
                else:
                    rect.br.x = 0
                giant_locations.append(rect)
            elif label == "qr_tag":
                qr_tags.append(rect)

        yolo_image_pub_msg = self.__cv_bridge.cv2_to_imgmsg(image, "bgr8")
        yolo_result = YoloResult(qr_tags=qr_tags, giant_locations=giant_locations)
        current_time = rospy.Time.now()
        yolo_image_pub_msg.header.stamp = current_time
        yolo_result.header.stamp = current_time
        if self.__frame_idx % 4 == 0:
            self.__yolo_image_mod_0_pub.publish(yolo_image_pub_msg)
            self.__yolo_result_mod_0_pub.publish(yolo_result)
        elif self.__frame_idx % 4 == 1:
            self.__yolo_image_mod_1_pub.publish(yolo_image_pub_msg)
            self.__yolo_result_mod_1_pub.publish(yolo_result)
        elif self.__frame_idx % 4 == 2:
            self.__yolo_image_mod_2_pub.publish(yolo_image_pub_msg)
            self.__yolo_result_mod_2_pub.publish(yolo_result)
        else:
            self.__yolo_image_mod_3_pub.publish(yolo_image_pub_msg)
            self.__yolo_result_mod_3_pub.publish(yolo_result)

        self.__frame_idx = self.__frame_idx + 1


def main(args):
    """!@brief The main function.
    @since 0.0.1
    """

    rospy.init_node("yolo_detection_node", anonymous=True)
    yolo_detection = YOLODetection()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
