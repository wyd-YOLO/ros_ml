/**
 * @file scandit_scanner.cpp
 * @author Nguyen Quang <quang@infiniumrobotics.com>
 * @brief The definitions of the ScanditScanner class.
 * @since 0.3.1
 * 
 * @copyright Copyright (c) 2018, Infinium Robotics, all rights reserved.
 * 
 */

#include "ros_ml/scandit_scanner.hpp"

ScanditScanner::ScanditScanner()
{
    // Print the current Scandit version
    printf("Scandit SDK Version: %s\n", SC_VERSION_STRING);

    // Create the recognition context.
    context_ptr_ = sc_recognition_context_new(SCANDIT_SDK_LICENSE_KEY, "/tmp", NULL);
    if (context_ptr_ == NULL)
    {
        printf("Could not initialize context.\n");
        return;
    }

    // Create barcode scanner QR code scan enabled.
    // The default preset is optimized for real-time frame processing using a camera
    settings_ptr_ = sc_barcode_scanner_settings_new_with_preset(SC_PRESET_NONE);
    if (settings_ptr_ == NULL)
    {
        sc_recognition_context_release(context_ptr_);
        return;
    }
    sc_barcode_scanner_settings_set_symbology_enabled(settings_ptr_, SC_SYMBOLOGY_QR, SC_TRUE);
    sc_barcode_scanner_settings_set_symbology_enabled(settings_ptr_, SC_SYMBOLOGY_EAN8, SC_TRUE);
    sc_barcode_scanner_settings_set_symbology_enabled(settings_ptr_, SC_SYMBOLOGY_CODE128, SC_TRUE);

    // Set the symbol count for CODE 128 symbology to be in range from 4 to 20.
    ScSymbologySettings* symbology_settings = sc_barcode_scanner_settings_get_symbology_settings(settings_ptr_, SC_SYMBOLOGY_CODE128);
    const uint16_t range_from = 4;
    const uint16_t range_to = 20;
    uint16_t symbology_count[16];
    for (int i = range_from; i < range_to; ++i)
    {
        symbology_count[i - range_from] = i;
    }
    sc_symbology_settings_set_active_symbol_counts(symbology_settings, symbology_count, 16);

    // We want to scan at most 10 code per frame.
    sc_barcode_scanner_settings_set_max_number_of_codes_per_frame(settings_ptr_, 10);

    // We define the centre area of the image to be the most likely location for a QR barcode.
    // ScRectangleF code_location = {0.1f, 0.2f, .80f, 0.6f};
    // sc_barcode_scanner_settings_set_code_location_area_2d(settings_ptr_, code_location);

    // Search in the full image but occasionally check the code loaction too.
    sc_barcode_scanner_settings_set_code_location_constraint_2d(settings_ptr_, SC_CODE_LOCATION_HINT);

    // Only keep codes for one frame and do not accumulate anything.
    sc_barcode_scanner_settings_set_code_duplicate_filter(settings_ptr_, 0);
    sc_barcode_scanner_settings_set_code_caching_duration(settings_ptr_, 0);

    // Our camera has no auto-focus.
    sc_barcode_scanner_settings_set_focus_mode(settings_ptr_, SC_CAMERA_FOCUS_MODE_FIXED);

    // The code is oriented horizontally in the image (left to right, or right to left).
    // sc_barcode_scanner_settings_set_code_direction_hint(settings_ptr_, SC_CODE_DIRECTION_HORIZONTAL);

    // Create a barcode scanner for our context and settings.
    scanner_ptr_ = sc_barcode_scanner_new_with_settings(context_ptr_, settings_ptr_);
    sc_barcode_scanner_settings_release(settings_ptr_);
    if (scanner_ptr_ == NULL)
    {
        sc_recognition_context_release(context_ptr_);
        return;
    }

    // Access the barcode scanner session. It collects all the results.
    session_ptr_ = sc_barcode_scanner_get_session(scanner_ptr_);

    // Signal a new frame sequence to the context.
    sc_recognition_context_start_new_frame_sequence(context_ptr_);
    image_description_ptr_ = sc_image_description_new();
}

ScanditScanner::~ScanditScanner()
{
    // Signal to the context that the frame sequence is finished.
    sc_recognition_context_end_frame_sequence(context_ptr_);

    // Cleanup all objects.
    sc_image_description_release(image_description_ptr_);
    sc_barcode_scanner_release(scanner_ptr_);
    sc_recognition_context_release(context_ptr_);
}

void ScanditScanner::set_image_description(const cv::Mat& scan_image)
{
    sc_image_description_set_layout(image_description_ptr_, SC_IMAGE_LAYOUT_GRAY_8U);
    sc_image_description_set_width(image_description_ptr_, scan_image.cols);
    sc_image_description_set_height(image_description_ptr_, scan_image.rows);
    sc_image_description_set_first_plane_offset(image_description_ptr_, 0);
    sc_image_description_set_first_plane_row_bytes(image_description_ptr_, scan_image.cols);
    sc_image_description_set_second_plane_row_bytes(image_description_ptr_, 0);
    sc_image_description_set_second_plane_offset(image_description_ptr_, 0);
    sc_image_description_set_memory_size(image_description_ptr_, scan_image.cols * scan_image.rows);
}

ScBarcodeArray* ScanditScanner::scan_image(const uint8_t* image_data)
{
    ScProcessFrameResult result = sc_recognition_context_process_frame(context_ptr_, image_description_ptr_, image_data);
    if (result.status != SC_RECOGNITION_CONTEXT_STATUS_SUCCESS)
    {
        printf("Processing frame failed with error %d: '%s'\n", result.status,
               sc_context_status_flag_get_message(result.status));
    }
    return sc_barcode_scanner_session_get_newly_recognized_codes(session_ptr_);
}
