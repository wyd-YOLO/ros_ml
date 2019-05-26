/**
 * @file tesseract_ocr.h
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the TesseractOCR class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef _TESSERACT_OCR_H_
#define _TESSERACT_OCR_H_

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
 * @brief A class to recognize text from YOLO detection output.
 * 
 * @since 0.0.1
 * 
 */
class TesseractOCR {
   private:
    ros::NodeHandle node;  //!< @brief The ros node handle. @since 0.0.1

    std::string yolo_image_topic;                                    //!< @brief The YOLO detection image topic. @since 0.0.1
    message_filters::Subscriber<sensor_msgs::Image> yolo_image_sub;  //!< @brief The YOLO detection image subscriber. @since 0.0.1

    std::string yolo_result_topic;                                    //!< @brief The YOLO detection result topic. @since 0.0.1
    message_filters::Subscriber<ros_ml::YoloResult> yolo_result_sub;  //!< @brief The YOLO detection result subscriber. @since 0.0.1

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ros_ml::YoloResult> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;  //!< @brief The Synchronizer shared ponter. @since 0.0.1

    cv::Ptr<cv::text::OCRTesseract> ocr_tesseract;  //!< @brief The OCRTesseract object. @since 0.0.1

    std::string tesseract_image_mod_topic;               //!< @brief The modularised TesseractOCR image topic. @since 0.0.1
    image_transport::Publisher tesseract_image_mod_pub;  //!< @brief The modularised TesseractOCR image publisher. @since 0.0.1

    std::string tesseract_result_mod_topic;   //!< @brief The modularised TesseractOCR result topic. @since 0.0.1
    ros::Publisher tesseract_result_mod_pub;  //!< @brief The modularised TesseractOCR result publisher. @since 0.0.1

   public:
    /**
     * @brief Construct a new TesseractOCR object.
     * 
     * @param[in] node The ros node handle.
     * @since 0.0.1
     */
    TesseractOCR(ros::NodeHandle node);

    /**
     * @brief Destroy the TesseractOCR object.
     * 
     * @since 0.0.1
     * 
     */
    ~TesseractOCR();

    /**
     * @brief The callback function.
     * 
     * @param[in] img_msg The YOLO detection image.
     * @param[in] result_msg The YOLO detection result.
     * @since 0.0.1
     */
    void callback(const sensor_msgs::ImageConstPtr& img_msg, const ros_ml::YoloResultConstPtr& result_msg);
};

#endif /* _TESSERACT_OCR_H_ */
