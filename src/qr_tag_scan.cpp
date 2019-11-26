/**
 * @file qr_tag_scan.cpp
 * @author Nguyen Quang <nqoptik@gmail.com>
 * @brief The definitions of the QRTagScan class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/qr_tag_scan.hpp"

QRTagScan::QRTagScan(const ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
{
    // Initialise the YOLO detection image subscriber
    node_handle_.param<std::string>("yolo_image_topic", yolo_image_topic_, "yolo_detection_image");
    yolo_image_subscriber_.subscribe(node_handle_, yolo_image_topic_, 1);

    // Initialise the YOLO detection result subscriber
    node_handle_.param<std::string>("yolo_result_topic", yolo_result_topic_, "yolo_detection_result");
    yolo_result_subscriber_.subscribe(node_handle_, yolo_result_topic_, 1);

    // Initialise the synchroniser
    synchroniser_ptr_.reset(new ApproximateSynchroniser(ApproximatePolicy(10), yolo_image_subscriber_, yolo_result_subscriber_));
    synchroniser_ptr_->registerCallback(boost::bind(&QRTagScan::yolo_callback, this, _1, _2));

    // Initialise the Scandit scanner
    is_first_frame_ = true;
    scandit_scanner_ptr_ = std::make_shared<ScanditScanner>();

    // Initialise the modularised QRTagScan image publisher
    node_handle_.param<std::string>("qr_tag_scan_image_topic", modularised_image_topic_, "qr_tag_scan_image");
    image_transport::ImageTransport image_transport(node_handle_);
    modularised_image_publisher_ = image_transport.advertise(modularised_image_topic_, 1);
}

QRTagScan::~QRTagScan()
{
    ros::shutdown();
}

void QRTagScan::yolo_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr, const ros_ml::YoloResult::ConstPtr& yolo_result_message_ptr)
{
    // Decode the image message
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(image_message_ptr, "mono8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", image_message_ptr->encoding.c_str());
    }

    for (size_t i = 0; i < yolo_result_message_ptr->qr_tags.size(); ++i)
    {
        ros_ml::YoloObject yolo_object = yolo_result_message_ptr->qr_tags[i];
        cv::Rect rect(cv::Point(yolo_object.tl.x, yolo_object.tl.y),
                      cv::Point(yolo_object.br.x, yolo_object.br.y));
        std::cout << rect << std::endl;
        rect.x = MAX(0, rect.x - 30);
        rect.y = MAX(0, rect.y - 30);
        rect.x = MIN(image.cols - 201, rect.x);
        rect.y = MIN(image.rows - 201, rect.y);
        rect.width = 200;
        rect.height = 200;
        cv::Mat cropped_image = image(rect).clone();
        // Create an image description that is reused for every frame.
        if (is_first_frame_ == true)
        {
            scandit_scanner_ptr_->set_image_description(cropped_image);
            is_first_frame_ = false;
        }
        ScBarcodeArray* new_codes = scandit_scanner_ptr_->scan_image((const uint8_t*)cropped_image.data);
        for (size_t j = 0; j < sc_barcode_array_get_size(new_codes); ++j)
        {
            const ScBarcode* code = sc_barcode_array_get_item_at(new_codes, j);
            ScByteArray data = sc_barcode_get_data(code);
            ScQuadrilateral code_location = sc_barcode_get_location(code);

            // Calculate the perimeter, and estimate the incentre of the code
            cv::Point top_left(code_location.top_left.x + rect.x, code_location.top_left.y + rect.y);
            cv::Point top_right(code_location.top_right.x + rect.x, code_location.top_right.y + rect.y);
            cv::Point bottom_right(code_location.bottom_right.x + rect.x, code_location.bottom_right.y + rect.y);
            cv::Point bottom_left(code_location.bottom_left.x + rect.x, code_location.bottom_left.y + rect.y);
            infinium_scan::draw_quadrangle(image, top_left, top_right, bottom_right, bottom_left, 255, 2, 8);
            cv::putText(image, data.str, bottom_right, CV_FONT_HERSHEY_COMPLEX_SMALL, 2, 255, 2, CV_AA);
        }
        cv::rectangle(image, rect, 255, 2, 8);
    }

    // Publish the QRTagScan image
    sensor_msgs::ImagePtr qr_tag_scan_image_message;
    qr_tag_scan_image_message = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    qr_tag_scan_image_message->header.stamp = image_message_ptr->header.stamp;
    modularised_image_publisher_.publish(qr_tag_scan_image_message);
}
