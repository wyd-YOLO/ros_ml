/**
 * @file ocr_tesseract.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the OCRTesseract class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/ocr_tesseract.h"

OCRTesseract::OCRTesseract(ros::NodeHandle node) {
    this->node = node;
    yolo_image_sub.subscribe(node, "/yolo_detection_image", 1);
    yolo_result_sub.subscribe(node, "/yolo_detection_result", 1);
    sync.reset(new Sync(MySyncPolicy(10), yolo_image_sub, yolo_result_sub));
    sync->registerCallback(boost::bind(&OCRTesseract::callback, this, _1, _2));
}

OCRTesseract::~OCRTesseract() {
}

void OCRTesseract::callback(const sensor_msgs::ImageConstPtr& img_msg, const ros_ml::YoloResultConstPtr& result_msg) {
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(img_msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", img_msg->encoding.c_str());
    }
    cv::imshow("image", frame);
    std::cout << result_msg->giant_locations.size() << std::endl;
    cv::waitKey(1);
}
