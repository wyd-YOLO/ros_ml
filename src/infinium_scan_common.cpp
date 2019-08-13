/**
 * @file infinium_scan_common.cpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief Some common functions for infinium scan.
 * @since 0.3.0
 * 
 * @copyright Copyright (c) 2018, Infinium Robotics, all rights reserved.
 * 
 */

#include "ros_ml/infinium_scan_common.hpp"

namespace infinium_scan
{
float get_euclid_distance(const cv::Point& point_1, const cv::Point& point_2)
{
    float d_x = point_1.x - point_2.x;
    float d_y = point_1.y - point_2.y;
    return sqrtf(d_x * d_x + d_y * d_y);
}

float get_quadrangle_perimeter(const cv::Point& top_left,
                               const cv::Point& top_right,
                               const cv::Point& bottom_right,
                               const cv::Point& bottom_left)
{
    float top_length = get_euclid_distance(top_left, top_right);
    float right_length = get_euclid_distance(top_right, bottom_right);
    float bottom_length = get_euclid_distance(bottom_left, bottom_right);
    float left_length = get_euclid_distance(top_left, bottom_left);
    return (top_length + right_length + bottom_length + left_length);
}

void draw_quadrangle(cv::Mat& image,
                     const cv::Point& point_1,
                     const cv::Point& point_2,
                     const cv::Point& point_3,
                     const cv::Point& point_4,
                     const cv::Scalar& colour,
                     const int& thickness,
                     const int& line_type)
{
    cv::line(image, point_1, point_2, colour, thickness, line_type);
    cv::line(image, point_2, point_3, colour, thickness, line_type);
    cv::line(image, point_3, point_4, colour, thickness, line_type);
    cv::line(image, point_4, point_1, colour, thickness, line_type);
}

cv::Point get_quadrangle_incentre(const cv::Point& point_1,
                                  const cv::Point& point_2,
                                  const cv::Point& point_3,
                                  const cv::Point& point_4)
{
    return cv::Point((point_1.x + point_2.x + point_3.x + point_4.x) / 4, (point_1.y + point_2.y + point_3.y + point_4.y) / 4);
}

void rotate_image(const cv::Mat& source, cv::Mat& destination, const int& rotation)
{
    if (rotation == 1)
    {
        cv::transpose(source, destination);
        cv::flip(destination, destination, 1);
    }
    else if (rotation == 2)
    {
        cv::flip(source, destination, -1);
    }
    else if (rotation == 3)
    {
        cv::transpose(source, destination);
        cv::flip(destination, destination, 0);
    }
    else
    {
        destination = source.clone();
    }
}

} // namespace infinium_scan
