/**
 * @file qr_tag_scan_node.cpp
 * @author Nguyen Quang <nqoptik@gmail.com>
 * @brief The qr tag scan node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/qr_tag_scan.hpp"

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
    ros::init(argc, argv, "qr_tag_scan_node");
    ros::NodeHandle node_handle("~");
    QRTagScan tesseract_ocr(node_handle);
    ros::spin();
    return 0;
}
