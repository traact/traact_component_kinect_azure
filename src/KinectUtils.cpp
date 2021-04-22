/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

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
    {"COLOR_MJPG",K4A_IMAGE_FORMAT_COLOR_MJPG},
    {"COLOR_NV12",K4A_IMAGE_FORMAT_COLOR_NV12},
    {"COLOR_YUY2",K4A_IMAGE_FORMAT_COLOR_YUY2},
    {"COLOR_BGRA32",K4A_IMAGE_FORMAT_COLOR_BGRA32},
    {"DEPTH16",K4A_IMAGE_FORMAT_DEPTH16},
    {"IR16",K4A_IMAGE_FORMAT_IR16},
    {"CUSTOM",K4A_IMAGE_FORMAT_CUSTOM}
};
std::map<std::string, k4a_fps_t> KinectUtils::k4a_fps = {
    {"5", K4A_FRAMES_PER_SECOND_5},
    {"15", K4A_FRAMES_PER_SECOND_15},
    {"30", K4A_FRAMES_PER_SECOND_30}
};
}