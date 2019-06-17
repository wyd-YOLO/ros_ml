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
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

/**
 * @brief A class to subscriber to an image topic and save images with specific format.
 * 
 * @since 0.0.1
 * 
 */
class ImageFormater {
   private:
    ros::NodeHandle node_;              //!< @brief The ros node handle. @since 0.0.1
    ros::Subscriber image_subscriber_;  //!< @brief The image subscriber. @since 0.0.1

    float frame_rate_;            //!< @brief The frame rate. @since 0.0.1
    std::string image_topic_;     //!< @brief The image topic. @since 0.0.1
    int rotation_;                //!< @brief The camera rotation in [0, 1, 2, 3] <-> [0, 90, 180, 270]. @since 0.0.1
    std::string save_directory_;  //!< @brief The saving dicrectory. @since 0.0.1
    int start_index_;             //!< @brief The start index. @since 0.0.1

   public:
    /**
     * @brief Construct a new ImageFormater object.
     * 
     * @param[in] node The ros node handle.
     * @since 0.0.1
     */
    ImageFormater(ros::NodeHandle node);

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
     * @param[in] src The input image.
     * @param[out] dst The destination image.
     * @param[in] rotation The rotation value.
     * @since 0.0.1
     */
    static void rotate_image(const cv::Mat src, cv::Mat& dst, const int rotation);

    /**
     * @brief The compressed image callback function.
     * 
     * @param[in] msg The image message.
     * @since 0.0.1
     */
    void compressed_image_callback(const sensor_msgs::CompressedImageConstPtr& msg);

    /**
     * @brief Get the frame rate.
     * 
     * @return float The frame rate.
     * @since 0.0.1
     */
    float get_frame_rate();
};

#endif  // IMAGE_FORMATER_HPP
