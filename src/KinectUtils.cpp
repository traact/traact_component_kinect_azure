/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include "KinectUtils.h"

namespace traact::component::vision {
std::map<std::string, k4a_wired_sync_mode_t> KinectUtils::k4a_wired_sync_mode = {
    {"STANDALONE", K4A_WIRED_SYNC_MODE_STANDALONE},
    {"MASTER", K4A_WIRED_SYNC_MODE_MASTER},
    {"SUBORDINATE", K4A_WIRED_SYNC_MODE_SUBORDINATE}
};

std::map<std::string, k4a_depth_mode_t> KinectUtils::k4a_depth_mode = {
    {"OFF", K4A_DEPTH_MODE_OFF},
    {"NFOV_2X2BINNED", K4A_DEPTH_MODE_NFOV_2X2BINNED},
    {"NFOV_UNBINNED", K4A_DEPTH_MODE_NFOV_UNBINNED},
    {"WFOV_2X2BINNED", K4A_DEPTH_MODE_WFOV_2X2BINNED},
    {"WFOV_UNBINNED", K4A_DEPTH_MODE_WFOV_UNBINNED},
    {"PASSIVE_IR", K4A_DEPTH_MODE_PASSIVE_IR}
};
std::map<std::string, k4a_color_resolution_t> KinectUtils::k4a_color_resolution = {
    {"OFF", K4A_COLOR_RESOLUTION_OFF},
    {"720P", K4A_COLOR_RESOLUTION_720P},
    {"1080P", K4A_COLOR_RESOLUTION_1080P},
    {"1440P", K4A_COLOR_RESOLUTION_1440P},
    {"1536P", K4A_COLOR_RESOLUTION_1536P},
    {"2160P", K4A_COLOR_RESOLUTION_2160P},
    {"3072P", K4A_COLOR_RESOLUTION_3072P}
};
std::map<std::string, k4a_image_format_t> KinectUtils::k4a_image_format = {
    {"COLOR_MJPG", K4A_IMAGE_FORMAT_COLOR_MJPG},
    {"COLOR_NV12", K4A_IMAGE_FORMAT_COLOR_NV12},
    {"COLOR_YUY2", K4A_IMAGE_FORMAT_COLOR_YUY2},
    {"COLOR_BGRA32", K4A_IMAGE_FORMAT_COLOR_BGRA32},
    {"DEPTH16", K4A_IMAGE_FORMAT_DEPTH16},
    {"IR16", K4A_IMAGE_FORMAT_IR16},
    {"CUSTOM", K4A_IMAGE_FORMAT_CUSTOM}
};
std::map<std::string, k4a_fps_t> KinectUtils::k4a_fps = {
    {"5", K4A_FRAMES_PER_SECOND_5},
    {"15", K4A_FRAMES_PER_SECOND_15},
    {"30", K4A_FRAMES_PER_SECOND_30}
};
}