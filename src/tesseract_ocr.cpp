/**
 * @file tesseract_ocr.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the TesseractOCR class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/tesseract_ocr.h"

TesseractOCR::TesseractOCR(ros::NodeHandle node) {
    this->node = node;

    // Initialize YOLO detection image subscriber
    node.param<std::string>("yolo_image_topic", yolo_image_topic, "yolo_detection_image");
    yolo_image_sub.subscribe(node, yolo_image_topic, 1);

    // Initialize YOLO detection result subscriber
    node.param<std::string>("yolo_result_topic", yolo_result_topic, "yolo_detection_result");
    yolo_result_sub.subscribe(node, yolo_result_topic, 1);

    // Initialize the synchronizer
    sync.reset(new Sync(MySyncPolicy(10), yolo_image_sub, yolo_result_sub));
    sync->registerCallback(boost::bind(&TesseractOCR::callback, this, _1, _2));

    // Initialize ORCTesseract
    ocr_tesseract = cv::text::OCRTesseract::create(NULL, "eng", "-0123456789", cv::text::OEM_DEFAULT, cv::text::PSM_SINGLE_BLOCK);

    // Initialize TesseractOCR result publisher
    node.param<std::string>("tesseract_result_topic", tesseract_result_topic, "tesseract_ocr_result");
    tesseract_result_pub = node.advertise<ros_ml::OCRResult>(tesseract_result_topic, 1);
}

TesseractOCR::~TesseractOCR() {
    ros::shutdown();
}

void TesseractOCR::callback(const sensor_msgs::ImageConstPtr& img_msg, const ros_ml::YoloResultConstPtr& result_msg) {
    // Decode the image message
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(img_msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", img_msg->encoding.c_str());
    }

    std::stringstream ss_pub;
    ros_ml::OCRResult ocr_result;
    // For each location got from YOLO detection
    for (int i_l = 0; i_l < result_msg->giant_locations.size(); i_l++) {
        std::string output;
        std::vector<cv::Rect> boxes;
        std::vector<std::string> words;
        std::vector<float> confidences;
        cv::Rect rect(cv::Point(result_msg->giant_locations[i_l].tl.x, result_msg->giant_locations[i_l].tl.y),
                      cv::Point(result_msg->giant_locations[i_l].br.x, result_msg->giant_locations[i_l].br.y));
        cv::Mat cropped_image = frame(rect);

        // Run OCRTesseract
        ocr_tesseract->run(cropped_image, output, &boxes, &words, &confidences, cv::text::OCR_LEVEL_WORD);

        // Filter the OCRTesseract output to get the location
        for (int w = 0; w < words.size(); w++) {
            std::string word = words[w];
            if (word.length() >= 8) {
                // Find the two positions of '-' letters
                int pos_1 = 0, pos_2 = 0;
                for (int i = 0; i < word.length(); i++) {
                    if (word.c_str()[i] == '-') {
                        pos_1 = i;
                        for (int j = i + 1; j < word.length(); j++) {
                            if (word.c_str()[j] == '-') {
                                pos_2 = j;
                                break;
                            }
                        }
                        break;
                    }
                }

                // If the two positions of '-' letters are separated by an one digit number
                // and there are more than 4 digits before the first position
                if (pos_2 - pos_1 == 2 && pos_1 >= 4) {
                    std::string result = word.substr(pos_1 - 4, 8);
                    bool good_result = true;
                    for (int i = 0; i < word.length(); i++) {
                        if (result.c_str()[i] == ' ') {
                            good_result = false;
                        }
                    }
                    if (good_result) {
                        ros_ml::OCRObject ocr_object;
                        ocr_object.yolo_confidence = result_msg->giant_locations[i_l].confidence;
                        ocr_object.ocr_confidence = confidences[w] / 100.0f;
                        ocr_object.data = word.substr(pos_1 - 4, 8);
                        ocr_object.tl.x = boxes[w].tl().x + result_msg->giant_locations[i_l].tl.x;
                        ocr_object.tl.y = boxes[w].tl().y + result_msg->giant_locations[i_l].tl.y;
                        ocr_object.br.x = boxes[w].br().x + result_msg->giant_locations[i_l].tl.x;
                        ocr_object.br.y = boxes[w].br().y + result_msg->giant_locations[i_l].tl.y;
                        ocr_result.giant_locations.push_back(ocr_object);
                    }
                }
            }
        }
    }
    ocr_result.header.stamp = ros::Time::now();
    tesseract_result_pub.publish(ocr_result);
}
