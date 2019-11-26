/**
 * @file tesseract_ocr_node.cpp
 * @author Nguyen Quang <nqoptik@gmail.com>
 * @brief The tesseract ocr node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/tesseract_ocr.hpp"

/**
 * @brief The main function.
 * 
 * @param[in] argc The argument count.
 * @param[in] argv The argument vector.
 * @return The status value.
 * @since 0.0.1
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "tesseract_orc");
    ros::NodeHandle node_handle("~");
    TesseractOCR tesseract_ocr(node_handle);
    ros::spin();
    return 0;
}
