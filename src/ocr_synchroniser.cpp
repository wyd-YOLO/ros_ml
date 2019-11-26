/**
 * @file ocr_synchroniser.cpp
 * @author Nguyen Quang <nqoptik@gmail.com>
 * @brief The definitions of the OCRSynchroniser class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/ocr_synchroniser.hpp"

OCRSynchroniser::OCRSynchroniser(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    // Initialise the modularised TesseractOCR image subscriber
    node_handle_.param<std::string>("tesseract_image_mod_topic", modularised_image_topic_, "tesseract_ocr_image_mod");
    modularised_image_subscriber_ = node_handle_.subscribe(modularised_image_topic_, 1, &OCRSynchroniser::tesseract_image_callback, this);
    latest_image_timestamp_.sec = 0;
    latest_image_timestamp_.nsec = 0;

    // Initialise the modularised TesseractOCR result subscriber
    node_handle_.param<std::string>("tesseract_result_mod_topic", modularised_result_topic_, "tesseract_ocr_result_mod");
    modularised_result_subscriber_ = node_handle_.subscribe(modularised_result_topic_, 1, &OCRSynchroniser::tesseract_result_callback, this);
    latest_result_timestamp_.sec = 0;
    latest_result_timestamp_.nsec = 0;

    // Initialise the synchronised TesseractOCR image publisher
    node_handle_.param<std::string>("tesseract_image_syn_topic", synchronised_image_topic_, "tesseract_ocr_image_syn");
    image_transport::ImageTransport image_transport(node_handle_);
    synchronised_image_publisher_ = image_transport.advertise(synchronised_image_topic_, 1);

    // Initialise the synchronised TesseractOCR result publisher
    node_handle_.param<std::string>("tesseract_result_syn_topic", synchronised_result_topic_, "tesseract_ocr_result_syn");
    synchronised_result_publisher_ = node_handle_.advertise<ros_ml::OCRResult>(synchronised_result_topic_, 1);
}

OCRSynchroniser::~OCRSynchroniser()
{
    ros::shutdown();
}

void OCRSynchroniser::tesseract_image_callback(const sensor_msgs::Image::ConstPtr& modularised_image_message_ptr)
{
    ros::Time current_image_stamp = modularised_image_message_ptr->header.stamp;
    if (current_image_stamp > latest_image_timestamp_)
    {
        synchronised_image_publisher_.publish(modularised_image_message_ptr);
        latest_image_timestamp_ = current_image_stamp;
    }
}

void OCRSynchroniser::tesseract_result_callback(const ros_ml::OCRResult::ConstPtr& modularised_result_message_ptr)
{
    ros::Time current_result_stamp = modularised_result_message_ptr->header.stamp;
    if (current_result_stamp > latest_result_timestamp_)
    {
        synchronised_result_publisher_.publish(modularised_result_message_ptr);
        latest_result_timestamp_ = current_result_stamp;
    }
}
