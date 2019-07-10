/**
 * @file image_rectifier_node.cpp
 * @author Nguyen Quang <nguyenquang.emailbox@gmail.com>
 * @brief The image rectifier node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/image_rectifier.hpp"

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
    ImageRectifier image_rectifier(node_handle);
    ros::spin();
    return 0;
}
