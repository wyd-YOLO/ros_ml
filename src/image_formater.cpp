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

ImageFormater::ImageFormater(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    // Innitialise the image subscriber
    node_handle_.param<std::string>("image_topic", image_topic_, "image_raw");
    image_subscriber_ = node_handle_.subscribe(image_topic_, 1, &ImageFormater::compressed_image_callback, this);

    // Initialise the frame rate
    node_handle_.param<float>("frame_rate", frame_rate_, 50.0f);

    // Initialise the camera rotation
    node_handle_.param<int>("rotation", rotation_, 0);

    // Initialise the file name start index
    node_handle_.param<int>("start_index", start_index_, 0);

    // Create the saving directory
    node_handle_.param<std::string>("save_directory", save_directory_, "images/");
    std::stringstream stringstream_mkdir;
    stringstream_mkdir << "mkdir -p " << save_directory_;
    if (!boost::filesystem::exists(save_directory_))
    {
        if (system(stringstream_mkdir.str().c_str()) == 0)
        {
            printf("\"%s\" directory has been created.\n", save_directory_.c_str());
        }
        else
        {
            printf("Couldn't create directory \"%s\".\n", save_directory_.c_str());
        }
    }
    else
    {
        printf("\"%s\" directory exists.\n", save_directory_.c_str());
    }
}

ImageFormater::~ImageFormater()
{
    ros::shutdown();
}

void ImageFormater::rotate_image(const cv::Mat& source, cv::Mat& destination, const int& rotation)
{
    if (rotation == 1)
    {
        cv::transpose(source, destination);
        cv::flip(destination, destination, 1);
    }
    else if (rotation == 2)
    {
        cv::flip(source, destination, -1);
    }
    else if (rotation == 3)
    {
        cv::transpose(source, destination);
        cv::flip(destination, destination, 0);
    }
    else
    {
        destination = source.clone();
    }
}

void ImageFormater::compressed_image_callback(const sensor_msgs::CompressedImage::ConstPtr& compressed_image_message_ptr)
{
    cv::Mat image;
    try
    {
        image = cv::imdecode(cv::Mat(compressed_image_message_ptr->data), 0);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not decode the image!");
    }
    if (!image.data)
    {
        return;
    }

    if (rotation_ != 0)
    {
        rotate_image(image, image, rotation_);
    }

    std::stringstream stringstream_image_path;
    stringstream_image_path << save_directory_;
    if (start_index_ < 10)
    {
        stringstream_image_path << "00000";
    }
    else if (start_index_ < 100)
    {
        stringstream_image_path << "0000";
    }
    else if (start_index_ < 1000)
    {
        stringstream_image_path << "000";
    }
    else if (start_index_ < 10000)
    {
        stringstream_image_path << "00";
    }
    else if (start_index_ < 100000)
    {
        stringstream_image_path << "0";
    }
    stringstream_image_path << start_index_ << ".jpg";
    ++start_index_;

    cv::namedWindow("image", CV_WINDOW_AUTOSIZE);
    cv::imshow("image", image);
    cv::imwrite(stringstream_image_path.str(), image);
    cv::waitKey(1);
}

float ImageFormater::get_frame_rate() const
{
    return frame_rate_;
}
