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

ImageRectifier::ImageRectifier(ros::NodeHandle node_handle)
    : node_handle_(node_handle) {
    // Initialise the distorted image subscriber
    node_handle_.param<std::string>("image_dis_topic", distorted_image_topic_, "image_raw/compressed");
    distorted_image_subscriber_ = node_handle_.subscribe(distorted_image_topic_, 1, &ImageRectifier::compressed_image_callback, this);

    // Initialise the synchronised TesseractOCR image publisher
    node_handle_.param<std::string>("image_rect_topic", rectified_image_topic_, "rectified_image");
    image_transport::ImageTransport image_transport(node_handle_);
    rectified_image_publisher_ = image_transport.advertise(rectified_image_topic_, 1);

    // Initialise camera matrix and distortion coefficients
    camera_matrix_ = (cv::Mat1d(3, 3) << 1006.431366, 0.000000, 701.688964, 0.000000, 1006.210496, 439.973480, 0.000000, 0.000000, 1.000000);
    distortion_coefficients_ = (cv::Mat1d(1, 5) << -0.403809, 0.134177, -0.001263, 0.000390, 0.000000);

    // Calculate the undistortion rectify maps
    cv::initUndistortRectifyMap(camera_matrix_, distortion_coefficients_, cv::Mat(), cv::Mat(), cv::Size(1280, 960), CV_16SC2, undistortion_map_1_, undistortion_map_2_);

    // Initialise the camera rotation
    node_handle_.param<int>("rotation", rotation_, 0);
}

ImageRectifier::~ImageRectifier() {
    ros::shutdown();
}

void ImageRectifier::compressed_image_callback(const sensor_msgs::CompressedImageConstPtr& compressed_image_message) {
    // Decode the image message
    cv::Mat image;
    try {
        image = cv::imdecode(cv::Mat(compressed_image_message->data), 0);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("ImageRectifier::compressed_image_callback: Could not decode the image message.");
    }
    if (!image.data) {
        printf("ImageRectifier::compressed_image_callback: Empty image data.\n");
        return;
    }

    // Rectify the image
    cv::Mat rectified_image;
    cv::remap(image, rectified_image, undistortion_map_1_, undistortion_map_2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Rotate the image
    if (rotation_ != 0) {
        ImageFormater::rotate_image(rectified_image, rectified_image, rotation_);
    }

    // Publish the rectified image
    sensor_msgs::ImagePtr rectified_image_message;
    rectified_image_message = cv_bridge::CvImage(std_msgs::Header(), "mono8", rectified_image).toImageMsg();
    rectified_image_message->header.stamp = compressed_image_message->header.stamp;
    rectified_image_publisher_.publish(rectified_image_message);
}
