/**
 * @file ocr_tesseract_node.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The ocr tesseract node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/ocr_tesseract.h"

/**
 * @brief The main function.
 * 
 * @param[in] argc The argument count.
 * @param[in] argv The argument vector.
 * @return The status value.
 * @since 0.0.1
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "orc_tesseract");
    ros::NodeHandle node("~");
    OCRTesseract ocr_tesseract(node);

    ros::spin();
    return 0;
}
