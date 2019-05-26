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

    // Initialize the YOLO detection image subscriber
    node.param<std::string>("yolo_image_topic", yolo_image_topic, "yolo_detection_image");
    yolo_image_sub.subscribe(node, yolo_image_topic, 1);

    // Initialize the YOLO detection result subscriber
    node.param<std::string>("yolo_result_topic", yolo_result_topic, "yolo_detection_result");
    yolo_result_sub.subscribe(node, yolo_result_topic, 1);

    // Initialize the synchronizer
    sync.reset(new Sync(MySyncPolicy(10), yolo_image_sub, yolo_result_sub));
    sync->registerCallback(boost::bind(&TesseractOCR::callback, this, _1, _2));

    // Initialize the ORCTesseract
    ocr_tesseract = cv::text::OCRTesseract::create(NULL, "eng", "-0123456789", cv::text::OEM_DEFAULT, cv::text::PSM_SINGLE_BLOCK);

    // Initialize the modularised TesseractOCR image publisher
    node.param<std::string>("tesseract_image_mod_topic", tesseract_image_mod_topic, "tesseract_ocr_image_mod");
    image_transport::ImageTransport it(node);
    tesseract_image_mod_pub = it.advertise(tesseract_image_mod_topic, 1);

    // Initialize the modularised TesseractOCR result publisher
    node.param<std::string>("tesseract_result_mod_topic", tesseract_result_mod_topic, "tesseract_ocr_result_mod");
    tesseract_result_mod_pub = node.advertise<ros_ml::OCRResult>(tesseract_result_mod_topic, 1);
}

TesseractOCR::~TesseractOCR() {
    ros::shutdown();
}

void TesseractOCR::callback(const sensor_msgs::ImageConstPtr& img_msg, const ros_ml::YoloResultConstPtr& result_msg) {
    // Decode the image message
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
    }

    // The TesseractOCR result
    ros_ml::OCRResult ocr_result;

    // For each location got from YOLO detection
    for (int i_l = 0; i_l < result_msg->giant_locations.size(); i_l++) {
        std::string output;
        std::vector<cv::Rect> boxes;
        std::vector<std::string> words;
        std::vector<float> confidences;
        cv::Rect rect(cv::Point(result_msg->giant_locations[i_l].tl.x, result_msg->giant_locations[i_l].tl.y),
                      cv::Point(result_msg->giant_locations[i_l].br.x, result_msg->giant_locations[i_l].br.y));
        cv::Mat cropped_image = frame(rect).clone();
        cv::cvtColor(cropped_image, cropped_image, cv::COLOR_BGR2GRAY);

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

    // Draw YOLO detection result for giant locations
    for (int i = 0; i < result_msg->giant_locations.size(); i++) {
        cv::Point pt1 = cv::Point(result_msg->giant_locations[i].tl.x, result_msg->giant_locations[i].tl.y);
        cv::Point pt2 = cv::Point(result_msg->giant_locations[i].br.x, result_msg->giant_locations[i].br.y);
        cv::rectangle(frame, pt1, pt2, cv::Scalar(255, 0, 0), 2);
    }

    // Draw YOLO detection result for qr tags
    for (int i = 0; i < result_msg->qr_tags.size(); i++) {
        cv::Point pt1 = cv::Point(result_msg->qr_tags[i].tl.x, result_msg->qr_tags[i].tl.y);
        cv::Point pt2 = cv::Point(result_msg->qr_tags[i].br.x, result_msg->qr_tags[i].br.y);
        cv::rectangle(frame, pt1, pt2, cv::Scalar(0, 0, 255), 2);
    }

    // Draw TesseractOCR result for giant locaitons
    for (int i = 0; i < ocr_result.giant_locations.size(); i++) {
        cv::Point pt1 = cv::Point(ocr_result.giant_locations[i].tl.x, ocr_result.giant_locations[i].tl.y);
        cv::Point pt2 = cv::Point(ocr_result.giant_locations[i].br.x, ocr_result.giant_locations[i].br.y);
        cv::rectangle(frame, pt1, pt2, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame, ocr_result.giant_locations[i].data, pt1, 1, 2, cv::Scalar(0, 255, 0), 2);
    }

    // Publish the TesseractOCR image
    sensor_msgs::ImagePtr ocr_img;
    ocr_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    ocr_img->header.stamp = img_msg->header.stamp;
    tesseract_image_mod_pub.publish(ocr_img);

    // Publish the TesseractOCR result
    ocr_result.header.stamp = img_msg->header.stamp;
    tesseract_result_mod_pub.publish(ocr_result);
}
