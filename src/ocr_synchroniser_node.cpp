/**
 * @file ocr_synchroniser_node.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The ocr synchroniser node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/ocr_synchroniser.hpp"

/**
 * @brief The main function.
 * 
 * @param[in] argc The argument count.
 * @param[in] argv The argument vector.
 * @return The status value.
 * @since 0.0.1
 */
int main(int argc, char** argv) {
    ros::init(argc, argv, "tesseract_orc");
    ros::NodeHandle node_handle("~");
    OCRSynchroniser ocr_synchroniser(node_handle);
    ros::spin();
    return 0;
}
