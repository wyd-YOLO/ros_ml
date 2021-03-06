/**
 * @file image_formater.hpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The header file of the ImageFormater class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#ifndef IMAGE_FORMATER_HPP
#define IMAGE_FORMATER_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

#include <opencv2/highgui/highgui.hpp>

/**
 * @brief A class to subscriber to an image topic and save images with specific format.
 * 
 * @since 0.0.1
 * 
 */
class ImageFormater
{
private:
    ros::NodeHandle node_handle_; //!< @brief The ros node handle. @since 0.0.1

    std::string image_topic_;          //!< @brief The image topic. @since 0.0.1
    ros::Subscriber image_subscriber_; //!< @brief The image subscriber. @since 0.0.1

    float frame_rate_; //!< @brief The frame rate. @since 0.0.1

    int rotation_; //!< @brief The camera rotation in [0, 1, 2, 3] <-> [0, 90, 180, 270]. @since 0.0.1

    int start_index_; //!< @brief The start index. @since 0.0.1

    std::string save_directory_; //!< @brief The saving dicrectory. @since 0.0.1

public:
    /**
     * @brief Construct a new ImageFormater object.
     * 
     * @param[in] node_handle The ros node handle.
     * @since 0.0.1
     */
    ImageFormater(const ros::NodeHandle& node_handle);

    /**
     * @brief Destroy the ImageFormater object.
     * 
     * @since 0.0.1
     * 
     */
    ~ImageFormater();

    /**
     * @brief Rotate a image by 90, 180, 270 degrees.
     * 
     * @param[in] source The input image.
     * @param[out] destination The destination image.
     * @param[in] rotation The rotation value.
     * @since 0.0.1
     */
    static void rotate_image(const cv::Mat& source, cv::Mat& destination, const int& rotation);

    /**
     * @brief The compressed image callback function.
     * 
     * @param[in] compressed_image_message_ptr The compressed image message.
     * @since 0.0.1
     */
    void compressed_image_callback(const sensor_msgs::CompressedImage::ConstPtr& compressed_image_message_ptr);

    /**
     * @brief Get the frame rate.
     * 
     * @return float The frame rate.
     * @since 0.0.1
     */
    float get_frame_rate() const;
};

#endif // IMAGE_FORMATER_HPP
