/**
 * @file image_formater.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the ImageFormater class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/image_formater.h"

ImageFormater::ImageFormater(ros::NodeHandle node) {
    this->node = node;
    node.param<float>("frame_rate", frame_rate, 50.0f);
    node.param<std::string>("image_topic", image_topic, "image_raw");
    node.param<int>("rotation", rotation, 0);
    node.param<std::string>("save_directory", save_directory, "images/");
    node.param<int>("start_index", start_index, 0);

    // Create the saving directory
    std::stringstream ss_cp;
    ss_cp << "mkdir -p " << save_directory;
    if (!boost::filesystem::exists(save_directory)) {
        if (system(ss_cp.str().c_str()) == 0) {
            printf("\"%s\" directory has been created.\n", save_directory.c_str());
        } else {
            printf("Couldn't create directory \"%s\".\n", save_directory.c_str());
        }
    } else {
        printf("\"%s\" directory exists.\n", save_directory.c_str());
    }

    image_subscriber = node.subscribe(image_topic, 1, &ImageFormater::compressed_image_callback, this);
}

ImageFormater::~ImageFormater() {
    ros::shutdown();
}

void ImageFormater::rotate_image(const cv::Mat src, cv::Mat& dst, int rotation) {
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

    if (rotation != 0) {
        rotate_image(image, image, rotation);
    }

    std::stringstream ss;
    ss << save_directory;
    if (start_index < 10) {
        ss << "00000";
    } else if (start_index < 100) {
        ss << "0000";
    } else if (start_index < 1000) {
        ss << "000";
    } else if (start_index < 10000) {
        ss << "00";
    } else if (start_index < 100000) {
        ss << "0";
    }
    ss << start_index << ".jpg";
    start_index++;

    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", image);
    cv::imwrite(ss.str(), image);
    cv::waitKey(1);
}

float ImageFormater::get_frame_rate() {
    return frame_rate;
}
