/**
 * @file image_formater.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the ImageFormater class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/image_formater.hpp"

ImageFormater::ImageFormater(ros::NodeHandle node)
    : node_(node) {
    node_.param<float>("frame_rate", frame_rate_, 50.0f);
    node_.param<std::string>("image_topic", image_topic_, "image_raw");
    node_.param<int>("rotation", rotation_, 0);
    node_.param<std::string>("save_directory", save_directory_, "images/");
    node_.param<int>("start_index", start_index_, 0);

    // Create the saving directory
    std::stringstream ss_cp;
    ss_cp << "mkdir -p " << save_directory_;
    if (!boost::filesystem::exists(save_directory_)) {
        if (system(ss_cp.str().c_str()) == 0) {
            printf("\"%s\" directory has been created.\n", save_directory_.c_str());
        } else {
            printf("Couldn't create directory \"%s\".\n", save_directory_.c_str());
        }
    } else {
        printf("\"%s\" directory exists.\n", save_directory_.c_str());
    }

    image_subscriber_ = node_.subscribe(image_topic_, 1, &ImageFormater::compressed_image_callback, this);
}

ImageFormater::~ImageFormater() {
    ros::shutdown();
}

void ImageFormater::rotate_image(const cv::Mat src, cv::Mat& dst, const int rotation) {
    if (rotation == 1) {
        cv::transpose(src, dst);
        cv::flip(dst, dst, 1);
    } else if (rotation == 2) {
        cv::flip(src, dst, -1);
    } else if (rotation == 3) {
        cv::transpose(src, dst);
        cv::flip(dst, dst, 0);
    } else {
        dst = src.clone();
    }
}

void ImageFormater::compressed_image_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
    cv::Mat image;
    try {
        image = cv::imdecode(cv::Mat(msg->data), 0);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not decode the image!");
    }
    if (!image.data) {
        return;
    }

    if (rotation_ != 0) {
        rotate_image(image, image, rotation_);
    }

    std::stringstream ss;
    ss << save_directory_;
    if (start_index_ < 10) {
        ss << "00000";
    } else if (start_index_ < 100) {
        ss << "0000";
    } else if (start_index_ < 1000) {
        ss << "000";
    } else if (start_index_ < 10000) {
        ss << "00";
    } else if (start_index_ < 100000) {
        ss << "0";
    }
    ss << start_index_ << ".jpg";
    ++start_index_;

    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", image);
    cv::imwrite(ss.str(), image);
    cv::waitKey(1);
}

float ImageFormater::get_frame_rate() {
    return frame_rate_;
}
