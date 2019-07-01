/**
 * @file tesseract_ocr.hpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the TesseractOCR class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef TESSERACT_OCR_HPP
#define TESSERACT_OCR_HPP

#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/text.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros_ml/YoloResult.h"
#include "ros_ml/OCRResult.h"

/**
 * @brief A class to recognise text from YOLO detection output.
 * 
 * @since 0.0.1
 * 
 */
class TesseractOCR {
   private:
    ros::NodeHandle node_handle_;  //!< @brief The ros node handle. @since 0.0.1

    std::string yolo_image_topic_;                                           //!< @brief The YOLO detection image topic. @since 0.0.1
    message_filters::Subscriber<sensor_msgs::Image> yolo_image_subscriber_;  //!< @brief The YOLO detection image subscriber. @since 0.0.1

    std::string yolo_result_topic_;                                           //!< @brief The YOLO detection result topic. @since 0.0.1
    message_filters::Subscriber<ros_ml::YoloResult> yolo_result_subscriber_;  //!< @brief The YOLO detection result subscriber. @since 0.0.1

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ros_ml::YoloResult> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSynchroniser;
    boost::shared_ptr<ApproximateSynchroniser> synchroniser_ptr_;  //!< @brief The Synchroniser shared ponter. @since 0.0.1

    cv::Ptr<cv::text::OCRTesseract> ocr_tesseract_ptr_;  //!< @brief The OCRTesseract object. @since 0.0.1

    std::string modularised_image_topic_;                     //!< @brief The modularised TesseractOCR image topic. @since 0.0.1
    image_transport::Publisher modularised_image_publisher_;  //!< @brief The modularised TesseractOCR image publisher. @since 0.0.1

    std::string modularised_result_topic_;         //!< @brief The modularised TesseractOCR result topic. @since 0.0.1
    ros::Publisher modularised_result_publisher_;  //!< @brief The modularised TesseractOCR result publisher. @since 0.0.1

   public:
    /**
     * @brief Construct a new TesseractOCR object.
     * 
     * @param[in] node_handle The ros node handle.
     * @since 0.0.1
     */
    TesseractOCR(ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the TesseractOCR object.
     * 
     * @since 0.0.1
     * 
     */
    ~TesseractOCR();

    /**
     * @brief The YOLO callback function.
     * 
     * @param[in] image_message_ptr The YOLO detection image.
     * @param[in] yolo_result_message_ptr The YOLO detection result.
     * @since 0.0.1
     */
    void yolo_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr, const ros_ml::YoloResult::ConstPtr& yolo_result_message_ptr);
};

#endif  // TESSERACT_OCR_HPP
