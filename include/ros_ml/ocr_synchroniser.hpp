/**
 * @file ocr_synchroniser.hpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the OCRSynchroniser class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef OCR_SYNCHRONISER_HPP
#define OCR_SYNCHRONISER_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "ros_ml/OCRResult.h"
#include "ros_ml/YoloResult.h"

/**
 * @brief A class to synchronse the modularised TesseractOCR result.
 * 
 * @since 0.0.1
 * 
 */
class OCRSynchroniser
{
private:
    ros::NodeHandle node_handle_; //!< @brief The ros node handle. @since 0.0.1

    std::string modularised_image_topic_;          //!< @brief The modularised TesseractOCR image topic. @since 0.0.1
    ros::Subscriber modularised_image_subscriber_; //!< @brief The modularised TesseractOCR image subscriber. @since 0.0.1
    ros::Time latest_image_timestamp_;             //!< @brief The latest timestamp of the TesseractOCR image. @since 0.0.1

    std::string modularised_result_topic_;          //!< @brief The modularised TesseractOCR result topic. @since 0.0.1
    ros::Subscriber modularised_result_subscriber_; //!< @brief The modularised TesseractOCR result subscriber. @since 0.0.1
    ros::Time latest_result_timestamp_;             //!< @brief The latest timestamp of the TesseractOCR result. @since 0.0.1

    std::string synchronised_image_topic_;                    //!< @brief The synchronised TesseractOCR image topic. @since 0.0.1
    image_transport::Publisher synchronised_image_publisher_; //!< @brief The synchronised TesseractOCR image publisher. @since 0.0.1

    std::string synchronised_result_topic_;        //!< @brief The synchronised TesseractOCR result topic. @since 0.0.1
    ros::Publisher synchronised_result_publisher_; //!< @brief The synchronised TesseractOCR result publisher. @since 0.0.1

public:
    /**
     * @brief Construct a new OCRSynchroniser object.
     * 
     * @param[in] node_handle The ros node handle.
     * @since 0.0.1
     */
    OCRSynchroniser(const ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the OCRSynchroniser object.
     * 
     * @since 0.0.1
     * 
     */
    ~OCRSynchroniser();

    /**
     * @brief The TesseractOCR image synchroniser.
     * 
     * @param[in] modularised_image_message_ptr The modularised TesseractOCR image.
     * @since 0.0.1
     */
    void tesseract_image_callback(const sensor_msgs::Image::ConstPtr& modularised_image_message_ptr);

    /**
     * @brief The TesseractOCR image synchroniser.
     * 
     * @param[in] modularised_result_message_ptr The modularised TesseractOCR result.
     * @since 0.0.1
     */
    void tesseract_result_callback(const ros_ml::OCRResult::ConstPtr& modularised_result_message_ptr);
};

#endif // OCR_SYNCHRONISER_HPP
