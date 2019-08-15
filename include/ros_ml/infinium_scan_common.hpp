/**
 * @file infinium_scan_common.hpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief Some common functions for infinium scan.
 * @since 0.3.0
 * 
 * @copyright Copyright (c) 2018, Infinium Robotics, all rights reserved.
 * 
 */

#ifndef INFINIUM_SCAN_COMMON_HPP
#define INFINIUM_SCAN_COMMON_HPP

#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief The scandit SDK license key.
 * 
 * @since 0.3.0
 * 
 */
#define SCANDIT_SDK_LICENSE_KEY "Af2ckdyBSMzXI1dqAjc6czUd1ZiIEBmusUgdXMN1nOT0MDumS1yRzltyYGI9Yecv2Tu4I3Zq/XQ0cU4Ikz24CKplsPXXaCEY4S1RI9xxUgXAPv7MhBvknk04jTe59R/oNrGE0zBIExaLJBx+rzbmtMAzuQTJFoqcltGgMDjtOIOLdi0S3dq8b9z5pccqGFux9WWMMgMLyqUtijdcZ8O0g6DRVfR4//0tZO8ONcBIMXOLrubAKKusQW5eKyG7XEB22wjS1Ox+c5GtiO9f3zn8hPmwnT22Ea/E9X5GsFYXyKp1yEYtN0fynBFWdH2MnPgFk3/2vdHTqdtoicZXSfYBIkTdTsb6LBkGGr+RXpQo/2PNVaxuYR+tu1lVv+3xj0GDCw08gcFx9eMPCNMp4utZ+XpJEzDmKOj5gLXFFxMOK7nxv9AlM7UZUK+zFjH4Snjww+2S+25UdyJqy9B8RjfSfiPrZV+RVyA5Bf3WEc/4ZRAGFsofykt6Bp5YCXF+Z7fd3NRgppSdq0mpwP3JA7lvd6wq7WIhcYCt2RimKZJXl6puksqMFjKdsC23SBVOwa/CI3RXeON1NuXf6BCkVilfst9EJVnxrKJeNeynpyXxgDTIZbaxjRL9mOoPHG2YF1cwqEz7xMxhJfn+3dbSjSpaz6m43objY96XQNcpPRi9FXoxn+GAvVz6XPXMAFto0+0/0oUxNNn7r8h6QglM45f+yd+2Y/xHdzZRD7HWtAai3ow2jZKfXc6zI94ruwa1P5CAd7Z3tKgIOfdw4CBj7l5tfE1uXEJnLcaK9GGjFA=="

/**
 * @brief The infinium_scan namespace
 * 
 * @since 0.3.2
 * 
 */
namespace infinium_scan
{

/**
 * @brief This is to compute the Euclid distance between two given points.
 * 
 * @param[in] point_1 The first point.
 * @param[in] point_2 The second point.
 * @return The Euclid distance between two given points.
 * @since 0.3.0
 */
float get_euclid_distance(const cv::Point& point_1, const cv::Point& point_2);

/**
 * @brief This is to compute the perimeter of a quadrangle.
 * 
 * @param[in] top_left The coordinates of the top-left point.
 * @param[in] top_right The coordinates of the top-right point.
 * @param[in] bottom_right The coordinates of the bottom-right point.
 * @param[in] bottom_left The coordinates of the bottom-left point.
 * @return The perimeter of the given quadrangle.
 * @since 0.3.0
 */
float get_quadrangle_perimeter(const cv::Point& top_left,
                               const cv::Point& top_right,
                               const cv::Point& bottom_right,
                               const cv::Point& bottom_left);

/**
 * @brief This is to draw a quadrangle on an image.
 * 
 * @param[in, out] image The given image.
 * @param[in] point_1 The first point of the quadrangle.
 * @param[in] point_2 The second point of the quadrangle.
 * @param[in] point_3 The third point of the quadrangle.
 * @param[in] point_4 The fourth point of the quadrangle.
 * @param[in] colour The colour of the quadrangle.
 * @param[in] thickness The thickness of the quadrangle.
 * @param[in] line_type The type of line.
 * @since 0.3.0
 */
void draw_quadrangle(cv::Mat& image,
                     const cv::Point& point_1,
                     const cv::Point& point_2,
                     const cv::Point& point_3,
                     const cv::Point& point_4,
                     const cv::Scalar& colour,
                     const int& thickness,
                     const int& line_type);

/**
 * @brief Measure the incentre of a quadrangle.
 * 
 * @param[in] point_1 The first point of the quadrangle.
 * @param[in] point_2 The second point of the quadrangle.
 * @param[in] point_3 The third point of the quadrangle.
 * @param[in] point_4 The fourth point of the quadrangle.
 * @return The incentre of the given quadrangle.
 * @since 0.3.0
 */
cv::Point get_quadrangle_incentre(const cv::Point& point_1,
                                  const cv::Point& point_2,
                                  const cv::Point& point_3,
                                  const cv::Point& point_4);

/**
 * @brief Rotate a image by 90, 180, 270 degrees.
 * 
 * @param[in] source The input image.
 * @param[out] destination The destination image.
 * @param[in] rotation The rotation value.
 * @since 0.3.0
 */
void rotate_image(const cv::Mat& source, cv::Mat& destination, const int& rotation);

} // namespace infinium_scan

#endif // INFINIUM_SCAN_COMMON_HPP
