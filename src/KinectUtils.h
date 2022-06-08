/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTUTILS_H_
#define TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTUTILS_H_

#include <map>
#include <k4a/k4a.hpp>
#include <traact/vision_datatypes.h>
namespace traact::component::vision {

class KinectUtils {
 public:
    static std::map<std::string, k4a_wired_sync_mode_t> k4a_wired_sync_mode;
    static std::map<std::string, k4a_depth_mode_t> k4a_depth_mode;
    static std::map<std::string, k4a_color_resolution_t> k4a_color_resolution;
    static std::map<std::string, k4a_image_format_t> k4a_image_format;
    static std::map<std::string, k4a_fps_t> k4a_fps;

    static bool k4a2traact(const k4a_calibration_camera_t &params, traact::vision::CameraCalibration &out) {

        out.fx = static_cast<float>(params.intrinsics.parameters.param.fx);
        out.fy = static_cast<float>(params.intrinsics.parameters.param.fy);
        out.cx = static_cast<float>(params.intrinsics.parameters.param.cx);
        out.cy = static_cast<float>(params.intrinsics.parameters.param.cy);
        out.skew = 0;

        out.radial_distortion.resize(6);
        out.tangential_distortion.resize(2);

        out.radial_distortion[0] = static_cast<float>(params.intrinsics.parameters.param.k1);
        out.radial_distortion[1] = static_cast<float>(params.intrinsics.parameters.param.k2);
        out.radial_distortion[2] = static_cast<float>(params.intrinsics.parameters.param.k3);
        out.radial_distortion[3] = static_cast<float>(params.intrinsics.parameters.param.k4);
        out.radial_distortion[4] = static_cast<float>(params.intrinsics.parameters.param.k5);
        out.radial_distortion[5] = static_cast<float>(params.intrinsics.parameters.param.k6);
        out.tangential_distortion[0] = static_cast<float>(params.intrinsics.parameters.param.p1);
        out.tangential_distortion[1] = static_cast<float>(params.intrinsics.parameters.param.p2);

        out.width = params.resolution_width;
        out.height = params.resolution_height;
        return true;
    }

};

}

#endif //TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTUTILS_H_
