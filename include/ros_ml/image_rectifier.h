/**
 * @file image_rectifier.h
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the ImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef _IMAGE_RECTIFIER_H_
#define _IMAGE_RECTIFIER_H_

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "ros_ml/image_formater.h"

/**
 * @brief A class to rectify the distorted image.
 * 
 * @since 0.0.1
 * 
 */
class ImageRectifier {
   private:
    ros::NodeHandle node;  //!< @brief The ros node handle. @since 0.0.1

    std::string image_dis_topic;    //!< @brief The distorted image topic. @since 0.0.1
    ros::Subscriber image_dis_sub;  //!< @brief The distorted image subsriber. @since 0.0.1

    std::string image_rect_topic;               //!< @brief The rectified image topic. @since 0.0.1
    image_transport::Publisher image_rect_pub;  //!< @brief The rectified image publisher. @since 0.0.1

    cv::Mat came_matrix;                 //!< @brief The camera matrix. @since 0.0.1
    cv::Mat dis_coef;                    //!< @brief The camera distortion coefficients. @since 0.0.1
    cv::Mat undist_map_1, undist_map_2;  //!< @brief The undistortion rectify maps. @since 0.0.1
    int rotation;                        //!< @brief The camera rotation in [0, 1, 2, 3] <-> [0, 90, 180, 270]. @since 0.0.1

   public:
    /**
     * @brief Construct a new ImageRectifier object.
     * 
     * @param[in] node The ros node handle.
     * @since 0.0.1
     */
    ImageRectifier(ros::NodeHandle node);

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
     * @param[in] msg The image message.
     * @since 0.0.1
     */
    void compressed_image_callback(const sensor_msgs::CompressedImageConstPtr& msg);
};

#endif /* _IMAGE_RECTIFIER_H_ */
