/**
 * @file image_formater_node.cpp
 * @author Nguyen Quang <nqoptik@gmail.com>
 * @brief The image formater node.
 * @since 0.0.1
 * 
 * @copyright Copyright (c) 2019, Nguyen Quang, all rights reserved.
 * 
 */

#include "ros_ml/image_formater.hpp"

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
    ros::init(argc, argv, "image_formater");
    ros::NodeHandle node_handle("~");
    ImageFormater image_formater(node_handle);

    ros::Rate rate(image_formater.get_frame_rate());
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
