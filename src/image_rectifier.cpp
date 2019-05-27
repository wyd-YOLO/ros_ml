/**
 * @file image_rectifier.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the ImageRectifier class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/image_rectifier.h"

ImageRectifier::ImageRectifier(ros::NodeHandle node) {
    this->node = node;

    // Initialize the distorted image subscriber
    node.param<std::string>("image_dis_topic", image_dis_topic, "image_raw/compressed");
    image_dis_sub = node.subscribe(image_dis_topic, 1, &ImageRectifier::compressed_image_callback, this);

    // Initialize the synchronised TesseractOCR image publisher
    node.param<std::string>("image_rect_topic", image_rect_topic, "image_rect");
    image_transport::ImageTransport it(node);
    image_rect_pub = it.advertise(image_rect_topic, 1);

    // Initialize camera matrix and distortion coefficients
    came_matrix = (cv::Mat1d(3, 3) << 1006.431366, 0.000000, 701.688964, 0.000000, 1006.210496, 439.973480, 0.000000, 0.000000, 1.000000);
    dis_coef = (cv::Mat1d(1, 5) << -0.403809, 0.134177, -0.001263, 0.000390, 0.000000);

    // Calculate the undistortion rectify maps
    cv::initUndistortRectifyMap(came_matrix, dis_coef, cv::Mat(), cv::Mat(), cv::Size(1280, 960), CV_16SC2, undist_map_1, undist_map_2);

    // Initialize the camera rotation
    node.param<int>("rotation", rotation, 0);
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
    cv::remap(image, image_rect, undist_map_1, undist_map_2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

    // Rotate the image
    if (rotation != 0) {
        ImageFormater::rotate_image(image_rect, image_rect, rotation);
    }

    // Publish the rectified image
    sensor_msgs::ImagePtr image_rect_msg;
    image_rect_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_rect).toImageMsg();
    image_rect_msg->header.stamp = img_msg->header.stamp;
    image_rect_pub.publish(image_rect_msg);
}
