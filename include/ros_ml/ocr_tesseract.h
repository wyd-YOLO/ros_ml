/**
 * @file ocr_tesseract.h
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the OCRTesseract class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef _OCR_TESSERACT_H_
#define _OCR_TESSERACT_H_

#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ros_ml/YoloResult.h"

/**
 * @brief A class to recognize text from YOLO detection output.
 * 
 * @since 0.0.1
 * 
 */
class OCRTesseract {
   private:
    ros::NodeHandle node;                                             //!< @brief The ros node handle. @since 0.0.1
    message_filters::Subscriber<sensor_msgs::Image> yolo_image_sub;   //!< @brief The YOLO detection image. @since 0.0.1
    message_filters::Subscriber<ros_ml::YoloResult> yolo_result_sub;  //!< @brief The YOLO detection result. @since 0.0.1
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ros_ml::YoloResult> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;  //!< @brief The Synchronizer shared ponter. @since 0.0.1

   public:
    /**
     * @brief Construct a new OCRTesseract object.
     * 
     * @param[in] node The ros node handle.
     * @since 0.0.1
     */
    OCRTesseract(ros::NodeHandle node);

    /**
     * @brief Destroy the OCRTesseract object.
     * 
     * @since 0.0.1
     * 
     */
    ~OCRTesseract();

    /**
     * @brief The callback function.
     * 
     * @param[in] img_msg The YOLO detection image.
     * @param[in] result_msg The YOLO detection result.
     * @since 0.0.1
     */
    void callback(const sensor_msgs::ImageConstPtr& img_msg, const ros_ml::YoloResultConstPtr& result_msg);
};

#endif /* _OCR_TESSERACT_H_ */
