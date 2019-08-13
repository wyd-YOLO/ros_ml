/**
 * @file qr_tag_scan.hpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the QRTagScan class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef QR_TAG_SCAN_HPP
#define QR_TAG_SCAN_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/text.hpp>

#include "ros_ml/scandit_scanner.hpp"

#include "ros_ml/OCRResult.h"
#include "ros_ml/YoloResult.h"

/**
 * @brief A class to recognise text from YOLO detection output.
 * 
 * @since 0.0.1
 * 
 */
class QRTagScan
{
private:
    ros::NodeHandle node_handle_; //!< @brief The ros node handle. @since 0.0.1

    std::string yolo_image_topic_;                                          //!< @brief The YOLO detection image topic. @since 0.0.1
    message_filters::Subscriber<sensor_msgs::Image> yolo_image_subscriber_; //!< @brief The YOLO detection image subscriber. @since 0.0.1

    std::string yolo_result_topic_;                                          //!< @brief The YOLO detection result topic. @since 0.0.1
    message_filters::Subscriber<ros_ml::YoloResult> yolo_result_subscriber_; //!< @brief The YOLO detection result subscriber. @since 0.0.1

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, ros_ml::YoloResult> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSynchroniser;
    boost::shared_ptr<ApproximateSynchroniser> synchroniser_ptr_; //!< @brief The Synchroniser shared ponter. @since 0.0.1

    bool is_first_frame_;                                 //!< @brief The variable to check the first frame. @since 0.0.1
    std::shared_ptr<ScanditScanner> scandit_scanner_ptr_; //!< @brief The ScanditScanner object. @since 0.0.1

    std::string modularised_image_topic_;                    //!< @brief The modularised QRTagScan image topic. @since 0.0.1
    image_transport::Publisher modularised_image_publisher_; //!< @brief The modularised QRTagScan image publisher. @since 0.0.1

public:
    /**
     * @brief Construct a new QRTagScan object.
     * 
     * @param[in] node_handle The ros node handle.
     * @since 0.0.1
     */
    QRTagScan(const ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the QRTagScan object.
     * 
     * @since 0.0.1
     * 
     */
    ~QRTagScan();

    /**
     * @brief The YOLO callback function.
     * 
     * @param[in] image_message_ptr The YOLO detection image.
     * @param[in] yolo_result_message_ptr The YOLO detection result.
     * @since 0.0.1
     */
    void yolo_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr, const ros_ml::YoloResult::ConstPtr& yolo_result_message_ptr);
};

#endif // QR_TAG_SCAN_HPP
