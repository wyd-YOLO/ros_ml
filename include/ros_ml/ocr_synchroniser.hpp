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
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "ros_ml/YoloResult.h"
#include "ros_ml/OCRResult.h"

/**
 * @brief A class to synchronse the modularised TesseractOCR result.
 * 
 * @since 0.0.1
 * 
 */
class OCRSynchroniser {
   private:
    ros::NodeHandle node_;  //!< @brief The ros node handle. @since 0.0.1

    std::string tesseract_image_mod_topic_;    //!< @brief The modularised TesseractOCR image topic. @since 0.0.1
    ros::Subscriber tesseract_image_mod_sub_;  //!< @brief The modularised TesseractOCR image subscriber. @since 0.0.1
    ros::Time latest_image_stamp_;             //!< @brief The latest timestamp of the TesseractOCR image. @since 0.0.1

    std::string tesseract_result_mod_topic_;    //!< @brief The modularised TesseractOCR result topic. @since 0.0.1
    ros::Subscriber tesseract_result_mod_sub_;  //!< @brief The modularised TesseractOCR result subscriber. @since 0.0.1
    ros::Time latest_result_stamp_;             //!< @brief The latest timestamp of the TesseractOCR result. @since 0.0.1

    std::string tesseract_image_syn_topic_;               //!< @brief The synchronised TesseractOCR image topic. @since 0.0.1
    image_transport::Publisher tesseract_image_syn_pub_;  //!< @brief The synchronised TesseractOCR image publisher. @since 0.0.1

    std::string tesseract_result_syn_topic_;   //!< @brief The synchronised TesseractOCR result topic. @since 0.0.1
    ros::Publisher tesseract_result_syn_pub_;  //!< @brief The synchronised TesseractOCR result publisher. @since 0.0.1

   public:
    /**
     * @brief Construct a new OCRSynchroniser object.
     * 
     * @param[in] node The ros node handle.
     * @since 0.0.1
     */
    OCRSynchroniser(ros::NodeHandle node);

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
     * @param[in] img_msg The modularised TesseractOCR image.
     * @since 0.0.1
     */
    void tesseract_image_callback(const sensor_msgs::ImageConstPtr& img_msg);

    /**
     * @brief The TesseractOCR image synchroniser.
     * 
     * @param[in] result_msg The modularised TesseractOCR result.
     * @since 0.0.1
     */
    void tesseract_result_callback(const ros_ml::OCRResultConstPtr& result_msg);
};

#endif  // OCR_SYNCHRONISER_HPP
