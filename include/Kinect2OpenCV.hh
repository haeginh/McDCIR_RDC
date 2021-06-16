#pragma once
// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.
#include <algorithm>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <limits>

#include <k4a/k4a.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using std::cerr;
using std::cout;
using std::endl;
using std::vector;



// The following functions provide the configurations that should be used for each camera.
// NOTE: For best results both cameras should have the same configuration (framerate, resolution, color and depth
// modes). Additionally the both master and subordinate should have the same exposure and power line settings. Exposure
// settings can be different but the subordinate must have a longer exposure from master. To synchronize a master and
// subordinate with different exposures the user should set `subordinate_delay_-d=0 -dp=detector_params.ymloff_master_usec = ((subordinate exposure
// time) - (master exposure time))/2`.
//
static k4a_device_configuration_t get_default_config()
{
    k4a_device_configuration_t camera_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    camera_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    camera_config.color_resolution = K4A_COLOR_RESOLUTION_2160P;
    camera_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; // No need for depth during calibration
    camera_config.camera_fps = K4A_FRAMES_PER_SECOND_15;     // Don't use all USB bandwidth
    return camera_config;
}

static cv::Mat color_to_opencv(const k4a_image_t im)
{
    cv::Mat cv_image_with_alpha(k4a_image_get_height_pixels(im), k4a_image_get_width_pixels(im), CV_8UC4, (void*)k4a_image_get_buffer(im));
    cv::Mat cv_image_no_alpha;
    cv::cvtColor(cv_image_with_alpha, cv_image_no_alpha, cv::COLOR_BGRA2BGR);
    return cv_image_no_alpha;
}

static cv::Mat depth_to_opencv(const k4a::image& im)
{
    return cv::Mat(im.get_height_pixels(),
        im.get_width_pixels(),
        CV_16U,
        (void*)im.get_buffer(),
        static_cast<size_t>(im.get_stride_bytes()));
}

static k4a_image_t color_to_depth(k4a_transformation_t transformation_handle,
                                       const k4a_image_t depth_image,
                                       const k4a_image_t color_image)
{
    int depth_image_width_pixels = k4a_image_get_width_pixels(depth_image);
    int depth_image_height_pixels = k4a_image_get_height_pixels(depth_image);
    k4a_image_t transformed_color_image = NULL;
    if (K4A_RESULT_SUCCEEDED != k4a_image_create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
                                                 depth_image_width_pixels,
                                                 depth_image_height_pixels,
                                                 depth_image_width_pixels * 4 * (int)sizeof(uint8_t),
                                                 &transformed_color_image))
    {
        printf("Failed to create transformed color image\n");
        exit(1);//return false;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_transformation_color_image_to_depth_camera(transformation_handle,
                                                                               depth_image,
                                                                               color_image,
                                                                               transformed_color_image))
    {
        printf("Failed to compute transformed color image\n");
        exit(1);//return false;
    }

    return transformed_color_image;
}

static k4a::image create_depth_image_like(const k4a::image& im)
{
    return k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16,
        im.get_width_pixels(),
        im.get_height_pixels(),
        im.get_width_pixels() * static_cast<int>(sizeof(uint16_t)));
}

static k4a::image create_color_image_like(const k4a::image& im)
{
    return k4a::image::create(K4A_IMAGE_FORMAT_COLOR_BGRA32,
        im.get_width_pixels(),
        im.get_height_pixels(),
        im.get_width_pixels() * 4 * static_cast<int>(sizeof(uint8_t)));
}


