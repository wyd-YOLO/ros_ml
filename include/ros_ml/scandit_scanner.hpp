/**
 * @file scandit_scanner.hpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The header file of the ScanditScanner class.
 * @since 0.3.0
 * 
 * @copyright Copyright (c) 2018, Infinium Robotics, all rights reserved.
 * 
 */

#ifndef SCANDIT_SCANNER_HPP
#define SCANDIT_SCANNER_HPP

#include <Scandit/ScBarcodeScanner.h>

#include "ros_ml/infinium_scan_common.hpp"

/**
 * @brief Class to scan barcode on images using scandit SDK.
 * 
 * @since 0.3.0
 * 
 */
class ScanditScanner
{
private:
    ScRecognitionContext* context_ptr_;         //!< @brief The scandit context. @since 0.3.0
    ScBarcodeScannerSettings* settings_ptr_;    //!< @brief The scandit settings. @since 0.3.0
    ScBarcodeScanner* scanner_ptr_;             //!< @brief The scandit scanner. @since 0.3.0
    ScBarcodeScannerSession* session_ptr_;      //!< @brief The scan session. @since 0.3.0
    ScImageDescription* image_description_ptr_; //!< @brief The image description. @since 0.3.0

public:
    /**
     * @brief Construct a new ScanditScanner object.
     * 
     * @since 0.3.0
     * 
     */
    ScanditScanner();

    /**
     * @brief Destroy the ScanditScanner object.
     * 
     * @since 0.3.0
     * 
     */
    ~ScanditScanner();

    /**
     * @brief Set the image description of object.
     * 
     * @param[in] scan_image The scan image.
     * @since 0.3.0
     */
    void set_image_description(const cv::Mat& scan_image);

    /**
     * @brief Scan the image.
     * 
     * @param[in] image_data The image data.
     * @return The scandit barcode scan results.
     * @since 0.3.0
     */
    ScBarcodeArray* scan_image(const uint8_t* image_data);
};

#endif // SCANDIT_SCANNER_HPP
