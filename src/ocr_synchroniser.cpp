/**
 * @file ocr_synchroniser.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the OCRSynchroniser class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/ocr_synchroniser.hpp"

OCRSynchroniser::OCRSynchroniser(ros::NodeHandle node)
    : node_(node) {
    // Initialise the modularised TesseractOCR image subscriber
    node_.param<std::string>("tesseract_image_mod_topic", tesseract_image_mod_topic_, "tesseract_ocr_image_mod");
    tesseract_image_mod_sub_ = node_.subscribe(tesseract_image_mod_topic_, 1, &OCRSynchroniser::tesseract_image_callback, this);
    latest_image_stamp_.sec = 0;
    latest_image_stamp_.nsec = 0;

    // Initialise the modularised TesseractOCR result subscriber
    node_.param<std::string>("tesseract_result_mod_topic", tesseract_result_mod_topic_, "tesseract_ocr_result_mod");
    tesseract_result_mod_sub_ = node_.subscribe(tesseract_result_mod_topic_, 1, &OCRSynchroniser::tesseract_result_callback, this);
    latest_result_stamp_.sec = 0;
    latest_result_stamp_.nsec = 0;

    // Initialise the synchronised TesseractOCR image publisher
    node_.param<std::string>("tesseract_image_syn_topic", tesseract_image_syn_topic_, "tesseract_ocr_image_syn");
    image_transport::ImageTransport it(node_);
    tesseract_image_syn_pub_ = it.advertise(tesseract_image_syn_topic_, 1);

    // Initialise the synchronised TesseractOCR result publisher
    node_.param<std::string>("tesseract_result_syn_topic", tesseract_result_syn_topic_, "tesseract_ocr_result_syn");
    tesseract_result_syn_pub_ = node_.advertise<ros_ml::OCRResult>(tesseract_result_syn_topic_, 1);
}

OCRSynchroniser::~OCRSynchroniser() {
    ros::shutdown();
}

void OCRSynchroniser::tesseract_image_callback(const sensor_msgs::ImageConstPtr& img_msg) {
    ros::Time current_image_stamp = img_msg->header.stamp;
    if (current_image_stamp > latest_image_stamp_) {
        // Decode the image message
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
        }

        // Publish the TesseractOCR image
        sensor_msgs::ImagePtr ocr_img;
        ocr_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        ocr_img->header.stamp = img_msg->header.stamp;
        tesseract_image_syn_pub_.publish(ocr_img);

        // Update the latest timestamp of the TesseractOCR image
        latest_image_stamp_ = current_image_stamp;
    }
}

void OCRSynchroniser::tesseract_result_callback(const ros_ml::OCRResultConstPtr& result_msg) {
    ros::Time current_result_stamp = result_msg->header.stamp;
    if (current_result_stamp > latest_result_stamp_) {
        // Publish the TesseractOCR result
        ros_ml::OCRResult ocr_result = *result_msg;
        tesseract_result_syn_pub_.publish(ocr_result);

        // Update the latest timestamp of the TesseractOCR result
        latest_result_stamp_ = current_result_stamp;
    }
}
