/**
 * @file image_rectifier.hpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the ImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef IMAGE_RECTIFIER_HPP
#define IMAGE_RECTIFIER_HPP

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ros_ml/image_formater.hpp"

/**
 * @brief A class to rectify the distorted image.
 * 
 * @since 0.0.1
 * 
 */
class ImageRectifier
{
private:
    ros::NodeHandle node_handle_; //!< @brief The ros node handle. @since 0.0.1

    std::string distorted_image_topic_;          //!< @brief The distorted image topic. @since 0.0.1
    ros::Subscriber distorted_image_subscriber_; //!< @brief The distorted image subsriber. @since 0.0.1

    std::string rectified_image_topic_;                    //!< @brief The rectified image topic. @since 0.0.1
    image_transport::Publisher rectified_image_publisher_; //!< @brief The rectified image publisher. @since 0.0.1

    cv::Mat camera_matrix_;                           //!< @brief The camera matrix. @since 0.0.1
    cv::Mat distortion_coefficients_;                 //!< @brief The camera distortion coefficients. @since 0.0.1
    cv::Mat undistortion_map_1_, undistortion_map_2_; //!< @brief The undistortion rectify maps. @since 0.0.1
    int rotation_;                                    //!< @brief The camera rotation in [0, 1, 2, 3] <-> [0, 90, 180, 270]. @since 0.0.1

public:
    /**
     * @brief Construct a new ImageRectifier object.
     * 
     * @param[in] node_handle The ros node handle.
     * @since 0.0.1
     */
    ImageRectifier(const ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the ImageRectifier object.
     * 
     * @since 0.0.1
     * 
     */
    ~ImageRectifier();

    /**
     * @brief The compressed image callback function.
     * 
     * @param[in] compressed_image_message_ptr The image message.
     * @since 0.0.1
     */
    void compressed_image_callback(const sensor_msgs::CompressedImage::ConstPtr& compressed_image_message_ptr);
};

#endif // IMAGE_RECTIFIER_HPP
