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

#ifndef TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREBODYOUTPUT_H_
#define TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREBODYOUTPUT_H_

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>

#include "KinectAzure.h"
#include "KinectUtils.h"
#include <traact/spatialBody.h>
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>
namespace traact::component::vision {

class KinectAzureBodyOutput : public KinectAzureComponent {
 public:
  explicit KinectAzureBodyOutput(const std::string &name) : KinectAzureComponent(name) {

  }

  traact::pattern::Pattern::Ptr GetPattern() const override {
    traact::pattern::Pattern::Ptr
        pattern = getUncalibratedCameraPattern();
    pattern->name = "KinectAzureBodyOutput";

    std::set<std::string> colorResolution = {"OFF", "720P", "1080P", "1440P" ,"1536P", "2160P", "3072P"};
    pattern->addParameter("ColorResolution", "1080P", colorResolution);

    std::set<std::string> colorImageFormat = {"OFF", "COLOR_MJPG", "COLOR_NV12", "COLOR_YUY2" ,"COLOR_BGRA32"};
    pattern->addParameter("ColorImageFormat", "COLOR_BGRA32", colorImageFormat);

    std::set<std::string> frameRate = {"5", "15", "30"};
    pattern->addParameter("FrameRate", "COLOR_BGRA32", frameRate);

    std::set<std::string> depthMode = {"OFF", "NFOV_2X2BINNED", "NFOV_UNBINNED", "WFOV_2X2BINNED" ,"WFOV_UNBINNED", "PASSIVE_IR"};
    pattern->addParameter("DepthMode", "NFOV_UNBINNED", depthMode);

    pattern->addParameter("SyncedImagesOnly", "true", pattern::CommonParameterEnums::bool_enum);

    pattern->addParameter("DepthDelayOffColor_usec", (int32_t )0);

    std::set<std::string> syncModes = {"STANDALONE", "MASTER", "SUBORDINATE"};
    pattern->addParameter("HardwareSyncMode", "STANDALONE", syncModes);

    pattern->addParameter("SubordinateDelayOffMaster_usec", (int32_t )0);

    pattern->addParameter("DisableStreamingIndicator", "false", pattern::CommonParameterEnums::bool_enum);

    return pattern;
  }

  bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
    std::shared_ptr<KinectAzureModule> k4a_module = std::dynamic_pointer_cast<KinectAzureModule>(module_);

    pattern::setValueFromParameter(parameter, "ColorResolution", k4a_module->device_configuration.color_resolution, "1080P", KinectUtils::k4a_color_resolution);
    pattern::setValueFromParameter(parameter, "ColorImageFormat", k4a_module->device_configuration.color_format, "COLOR_BGRA32", KinectUtils::k4a_image_format);
    pattern::setValueFromParameter(parameter, "FrameRate", k4a_module->device_configuration.camera_fps, "30", KinectUtils::k4a_fps);
    pattern::setValueFromParameter(parameter, "DepthMode", k4a_module->device_configuration.depth_mode, "NFOV_UNBINNED", KinectUtils::k4a_depth_mode);
    pattern::setBoolValueFromParameter(parameter, "SyncedImagesOnly", k4a_module->device_configuration.synchronized_images_only, true);
    pattern::setValueFromParameter(parameter, "DepthDelayOffColor_usec", k4a_module->device_configuration.depth_delay_off_color_usec, 0);
    pattern::setValueFromParameter(parameter, "HardwareSyncMode", k4a_module->device_configuration.wired_sync_mode, "STANDALONE", KinectUtils::k4a_wired_sync_mode);
    pattern::setValueFromParameter(parameter, "SubordinateDelayOffMaster_usec", k4a_module->device_configuration.subordinate_delay_off_master_usec, 0);
    pattern::setBoolValueFromParameter(parameter, "DisableStreamingIndicator", k4a_module->device_configuration.disable_streaming_indicator, false);

    k4a_module->tracker_configuration.gpu_device_id = 0;
    k4a_module->tracker_configuration.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
    k4a_module->tracker_configuration.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
    device_id_ = "Any";
  }

  KinectOutputs GetOutputType() override {
    return KinectOutputs::BodyTracking;
  }

  void process(k4abt::frame& body_frame, TimestampType ts) override {
    using namespace traact::spatial;

      auto buffer_future = request_callback_(ts);
      buffer_future.wait();
      auto buffer = buffer_future.get();
      if(!buffer){
          SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
          return;
      }

    size_t numBodies = body_frame.get_num_bodies();
    auto &newData = buffer->getOutput<BodyListHeader::NativeType, BodyListHeader>(0);
    newData.clear();
    for (size_t i = 0; i < numBodies; ++i) {
      k4abt_body_t k4abt_body = body_frame.get_body(i);

      Body body;
      body.id = i;

      for (unsigned int j = 0; j < K4ABT_JOINT_COUNT; ++j) {
        k4a_float3_t position = k4abt_body.skeleton.joints[j].position; // xyz
        k4a_quaternion_t orientation = k4abt_body.skeleton.joints[j].orientation; // wxyz
        k4abt_joint_confidence_level_t confidenceLevel = k4abt_body.skeleton.joints[j].confidence_level;

        BodyJoint bodyJoint;
        Eigen::Translation3d pos(position.v[0]/1000., -position.v[1]/1000., -position.v[2]/1000.);
        Eigen::Quaterniond rot(-orientation.v[0], -orientation.v[1], orientation.v[2], orientation.v[3]);
        bodyJoint.pose = pos * rot;
        bodyJoint.confidenceLevel = confidenceLevel;

        body.bodyJoints.emplace(static_cast<BodyJointType >(j), bodyJoint);
      }

      newData.emplace_back(std::move(body));
    }

      buffer->Commit(true);
  }

};
}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

using namespace rttr;
registration::class_<traact::component::vision::KinectAzureBodyOutput>("KinectAzureBodyOutput").constructor<std::string>()();
}

#endif //TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREBODYOUTPUT_H_
