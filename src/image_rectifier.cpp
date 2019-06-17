/**
 * @file image_rectifier.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the ImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/image_rectifier.hpp"

ImageRectifier::ImageRectifier(ros::NodeHandle node)
    : node_(node) {
    // Initialise the distorted image subscriber
    node_.param<std::string>("image_dis_topic", image_dis_topic_, "image_raw/compressed");
    image_dis_sub_ = node_.subscribe(image_dis_topic_, 1, &ImageRectifier::compressed_image_callback, this);

    // Initialise the synchronised TesseractOCR image publisher
    node_.param<std::string>("image_rect_topic", image_rect_topic_, "image_rect");
    image_transport::ImageTransport it(node_);
    image_rect_pub_ = it.advertise(image_rect_topic_, 1);

    // Initialise camera matrix and distortion coefficients
    camera_matrix_ = (cv::Mat1d(3, 3) << 1006.431366, 0.000000, 701.688964, 0.000000, 1006.210496, 439.973480, 0.000000, 0.000000, 1.000000);
    dis_coef_ = (cv::Mat1d(1, 5) << -0.403809, 0.134177, -0.001263, 0.000390, 0.000000);

    // Calculate the undistortion rectify maps
    cv::initUndistortRectifyMap(camera_matrix_, dis_coef_, cv::Mat(), cv::Mat(), cv::Size(1280, 960), CV_16SC2, undist_map_1_, undist_map_2_);

    // Initialise the camera rotation
    node_.param<int>("rotation", rotation_, 0);
}

ImageRectifier::~ImageRectifier() {
    ros::shutdown();
}

void ImageRectifier::compressed_image_callback(const sensor_msgs::CompressedImageConstPtr& img_msg) {
    // Decode the image message
    cv::Mat image;
    try {
        image = cv::imdecode(cv::Mat(img_msg->data), 0);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("ImageRectifier::compressed_image_callback: Could not decode the image message.");
    }
    if (!image.data) {
        printf("ImageRectifier::compressed_image_callback: Empty image data.\n");
        return;
    }

    // Rectify the image
    cv::Mat image_rect;
    cv::remap(image, image_rect, undist_map_1_, undist_map_2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Rotate the image
    if (rotation_ != 0) {
        ImageFormater::rotate_image(image_rect, image_rect, rotation_);
    }

    // Publish the rectified image
    sensor_msgs::ImagePtr image_rect_msg;
    image_rect_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rect).toImageMsg();
    image_rect_msg->header.stamp = img_msg->header.stamp;
    image_rect_pub_.publish(image_rect_msg);
}
