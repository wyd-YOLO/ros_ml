/**
 * @file tesseract_ocr.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the TesseractOCR class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/tesseract_ocr.hpp"

TesseractOCR::TesseractOCR(const ros::NodeHandle& node_handle)
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
    synchroniser_ptr_->registerCallback(boost::bind(&TesseractOCR::yolo_callback, this, _1, _2));

    // Initialise the ORCTesseract
    ocr_tesseract_ptr_ = cv::text::OCRTesseract::create(NULL, "eng", "-0123456789", cv::text::OEM_DEFAULT, cv::text::PSM_SINGLE_BLOCK);

    // Initialise the modularised TesseractOCR image publisher
    node_handle_.param<std::string>("tesseract_image_mod_topic", modularised_image_topic_, "tesseract_ocr_image_mod");
    image_transport::ImageTransport image_transport(node_handle_);
    modularised_image_publisher_ = image_transport.advertise(modularised_image_topic_, 1);

    // Initialise the debug image publisher
    image_transport::ImageTransport debug_image_transport(node_handle_);
    debug_image_publisher_ = debug_image_transport.advertise("debug_image", 1);

    // Initialise the modularised TesseractOCR result publisher
    node_handle_.param<std::string>("tesseract_result_mod_topic", modularised_result_topic_, "tesseract_ocr_result_mod");
    modularised_result_publisher_ = node_handle_.advertise<ros_ml::OCRResult>(modularised_result_topic_, 1);
}

TesseractOCR::~TesseractOCR()
{
    ros::shutdown();
}

void TesseractOCR::yolo_callback(const sensor_msgs::Image::ConstPtr& image_message_ptr, const ros_ml::YoloResult::ConstPtr& yolo_result_message_ptr)
{
    // Decode the image message
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(image_message_ptr, "bgr8")->image;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_message_ptr->encoding.c_str());
    }

    // The TesseractOCR result
    ros_ml::OCRResult ocr_result_message;

    // For each location got from the YOLO detection result
    for (size_t i_l = 0; i_l < yolo_result_message_ptr->giant_locations.size(); ++i_l)
    {
        std::string output;
        std::vector<cv::Rect> boxes;
        std::vector<std::string> words;
        std::vector<float> confidences;

        ros_ml::YoloObject yolo_object = yolo_result_message_ptr->giant_locations[i_l];
        // Find the black part in the location label
        cv::Rect original_rect(cv::Point(yolo_object.tl.x, yolo_object.tl.y),
                               cv::Point(yolo_object.br.x, yolo_object.br.y));
        cv::Mat original_location_image = image(original_rect).clone();
        cv::cvtColor(original_location_image, original_location_image, CV_BGR2GRAY);

        int black_box_size = std::min(original_location_image.rows * 3 / 4, original_location_image.cols);
        std::vector<int> intensity_vector(original_location_image.cols - black_box_size, 0);
        for (size_t i = 0; i < original_location_image.cols - black_box_size; ++i)
        {
            for (size_t j = 0; j < original_location_image.rows; ++j)
            {
                for (int k = 0; k < black_box_size; ++k)
                {
                    intensity_vector[i] += original_location_image.data[j * original_location_image.cols + i + k];
                }
            }
        }
        int min_element_index = std::min_element(intensity_vector.begin(), intensity_vector.end()) - intensity_vector.begin();
        std::cout << "\n";
        std::cout << min_element_index << " " << intensity_vector.size() << "\n";

        cv::Rect rect;
        if (min_element_index > 10 && min_element_index < original_location_image.cols - 10)
        {
            rect = cv::Rect(std::min(std::max(yolo_object.tl.x + min_element_index - original_location_image.rows * 3, 0), image.cols - min_element_index - 1), yolo_object.tl.y, original_location_image.rows * 3, original_location_image.rows);
        }
        else
        {
            rect = cv::Rect(cv::Point(yolo_object.tl.x, yolo_object.tl.y),
                            cv::Point(yolo_object.br.x, yolo_object.br.y));
        }
        cv::Mat cropped_image = image(rect).clone();
        cv::cvtColor(cropped_image, cropped_image, cv::COLOR_BGR2GRAY);
        cv::Mat canny_image;
        cv::Canny(cropped_image, canny_image, 25, 50);

        std::vector<cv::Vec4i> lines;
        cv::HoughLinesP(canny_image, lines, 1, CV_PI / 720, 50, 50, 3);
        std::sort(lines.begin(), lines.end(), [](const cv::Vec4i& line_1, const cv::Vec4i& line_2) {
            int d_1_x = line_1[2] - line_1[0];
            int d_1_y = line_1[3] - line_1[1];
            float length_1 = sqrt(d_1_x * d_1_x + d_1_y * d_1_y);
            int d_2_x = line_2[2] - line_2[0];
            int d_2_y = line_2[3] - line_2[1];
            float length_2 = sqrt(d_2_x * d_2_x + d_2_y * d_2_y);
            return (length_1 > length_2);
        });
        if (lines.size())
        {
            cv::line(canny_image, cv::Point(lines[0][0], lines[0][1]), cv::Point(lines[0][2], lines[0][3]), 255, 2, 8, 0);
            if (lines[0][0] > lines[0][2])
            {
                cv::Vec4i temporary(lines[0][2], lines[0][3], lines[0][0], lines[0][1]);
                lines[0] = temporary;
            }
            float d_x = lines[0][2] - lines[0][0];
            float d_y = lines[0][3] - lines[0][1];
            float length = sqrt(d_x * d_x + d_y * d_y);
            float cos_two_lines = d_x / length;
            if (cos_two_lines > 0.95)
            {
                float d_height = d_y / d_x * rect.width;
                cv::Point2f centre_point(rect.width / 2., rect.height / 2.);
                cos_two_lines = acos(cos_two_lines) * 180.0 / M_PI;
                cv::Mat rotation_matrix = cv::getRotationMatrix2D(centre_point, cos_two_lines, 1.0);
                cv::warpAffine(cropped_image, cropped_image, rotation_matrix, cropped_image.size());
            }
        }

        // Apply re-scaling for location label
        float ratio = 80.0f / cropped_image.rows;
        cv::resize(cropped_image, cropped_image, cv::Size(), ratio, ratio);
        size_t image_size = cropped_image.cols * cropped_image.rows;

        // Apply kmeans clustering with intensity is the criterion
        std::vector<float> intensities;
        intensities.reserve(image_size);
        for (size_t i = 0; i < image_size; ++i)
        {
            intensities.emplace_back(cropped_image.data[i]);
        }
        std::vector<int> labels;
        std::vector<uchar> centres;
        cv::kmeans(intensities, 2, labels, cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, 0.0001), 5, cv::KMEANS_PP_CENTERS, centres);
        std::sort(centres.begin(), centres.end());

        // Threshold the image with the intensities got from kmeans
        cv::threshold(cropped_image, cropped_image, centres[0] * 0.9 + centres[1] * 0.1, 255, CV_THRESH_BINARY_INV);

        // Publish the debug image
        sensor_msgs::ImagePtr debug_image_message;
        debug_image_message = cv_bridge::CvImage(std_msgs::Header(), "mono8", cropped_image).toImageMsg();
        debug_image_message->header.stamp = image_message_ptr->header.stamp;
        debug_image_publisher_.publish(debug_image_message);

        // Run the OCRTesseract
        ocr_tesseract_ptr_->run(cropped_image, output, &boxes, &words, &confidences, cv::text::OCR_LEVEL_WORD);

        // Filter the OCRTesseract output to get the location
        for (size_t w = 0; w < words.size(); ++w)
        {
            std::string word = words[w];
            if (word.length() >= 8)
            {
                // Find the two positions of '-' letters
                int first_position = 0, second_position = 0;
                for (size_t i = 0; i < word.length(); ++i)
                {
                    if (word.c_str()[i] == '-')
                    {
                        first_position = i;
                        for (size_t j = i + 1; j < word.length(); ++j)
                        {
                            if (word.c_str()[j] == '-')
                            {
                                second_position = j;
                                break;
                            }
                        }
                        break;
                    }
                }

                // If the two positions of '-' letters are separated by an one digit number
                // and there are more than 4 digits before the first position
                if (second_position - first_position == 2 && first_position >= 4)
                {
                    std::string result = word.substr(first_position - 4, 8);
                    bool is_a_good_result = true;
                    for (size_t i = 0; i < word.length(); ++i)
                    {
                        if (result.c_str()[i] == ' ')
                        {
                            is_a_good_result = false;
                        }
                    }
                    if (is_a_good_result)
                    {
                        ros_ml::OCRObject ocr_object;
                        ocr_object.yolo_confidence = yolo_object.confidence;
                        ocr_object.ocr_confidence = confidences[w] / 100.0f;
                        ocr_object.data = word.substr(first_position - 4, 8);
                        ocr_object.tl.x = boxes[w].tl().x + yolo_object.tl.x;
                        ocr_object.tl.y = boxes[w].tl().y + yolo_object.tl.y;
                        ocr_object.br.x = boxes[w].br().x + yolo_object.tl.x;
                        ocr_object.br.y = boxes[w].br().y + yolo_object.tl.y;
                        ocr_result_message.giant_locations.push_back(ocr_object);
                    }
                }
            }
        }
    }

    // Draw the YOLO detection result for giant locations
    for (size_t i = 0; i < yolo_result_message_ptr->giant_locations.size(); ++i)
    {
        cv::Point top_left = cv::Point(yolo_result_message_ptr->giant_locations[i].tl.x, yolo_result_message_ptr->giant_locations[i].tl.y);
        cv::Point bottom_right = cv::Point(yolo_result_message_ptr->giant_locations[i].br.x, yolo_result_message_ptr->giant_locations[i].br.y);
        cv::rectangle(image, top_left, bottom_right, cv::Scalar(255, 0, 0), 2);
    }

    // Draw the YOLO detection result for qr tags
    for (size_t i = 0; i < yolo_result_message_ptr->qr_tags.size(); ++i)
    {
        cv::Point top_left = cv::Point(yolo_result_message_ptr->qr_tags[i].tl.x, yolo_result_message_ptr->qr_tags[i].tl.y);
        cv::Point bottom_right = cv::Point(yolo_result_message_ptr->qr_tags[i].br.x, yolo_result_message_ptr->qr_tags[i].br.y);
        cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 0, 255), 2);
    }

    // Draw the TesseractOCR result for giant locaitons
    for (size_t i = 0; i < ocr_result_message.giant_locations.size(); ++i)
    {
        cv::Point top_left = cv::Point(ocr_result_message.giant_locations[i].tl.x, ocr_result_message.giant_locations[i].tl.y);
        cv::Point bottom_right = cv::Point(ocr_result_message.giant_locations[i].br.x, ocr_result_message.giant_locations[i].br.y);
        cv::rectangle(image, top_left, bottom_right, cv::Scalar(0, 255, 0), 2);
        cv::putText(image, ocr_result_message.giant_locations[i].data, top_left, 1, 2, cv::Scalar(0, 255, 0), 2);
    }

    // Publish the TesseractOCR image
    sensor_msgs::ImagePtr ocr_image_message;
    ocr_image_message = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    ocr_image_message->header.stamp = image_message_ptr->header.stamp;
    modularised_image_publisher_.publish(ocr_image_message);

    // Publish the TesseractOCR result
    ocr_result_message.header.stamp = image_message_ptr->header.stamp;
    modularised_result_publisher_.publish(ocr_result_message);
}
