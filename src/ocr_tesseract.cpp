/**
 * @file ocr_tesseract.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The definitions of the OCRTesseract class.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/ocr_tesseract.h"

OCRTesseract::OCRTesseract(ros::NodeHandle node) {
    this->node = node;
    yolo_image_sub.subscribe(node, "/yolo_detection_image", 1);
    yolo_result_sub.subscribe(node, "/yolo_detection_result", 1);
    sync.reset(new Sync(MySyncPolicy(10), yolo_image_sub, yolo_result_sub));
    sync->registerCallback(boost::bind(&OCRTesseract::callback, this, _1, _2));
    ocr_result_pub = node.advertise<std_msgs::String>("ocr_tesseract_result_pub", 1);
    ocr_tesseract = cv::text::OCRTesseract::create(NULL, "eng", "-0123456789", cv::text::OEM_DEFAULT, cv::text::PSM_SINGLE_BLOCK);
}

OCRTesseract::~OCRTesseract() {
}

void OCRTesseract::callback(const sensor_msgs::ImageConstPtr& img_msg, const ros_ml::YoloResultConstPtr& result_msg) {
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(img_msg, "mono8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", img_msg->encoding.c_str());
    }

    std::stringstream ss_pub;
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

        // Filter the ORCTesseract output to get the location
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
                        ss_pub << word.substr(pos_1 - 4, 8);
                    }
                }
            }
        }
    }
    std_msgs::String msg_string;
    msg_string.data = ss_pub.str();
    ocr_result_pub.publish(msg_string);
}
