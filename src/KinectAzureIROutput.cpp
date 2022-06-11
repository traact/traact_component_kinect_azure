/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREIROUTPUT_H_
#define TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREIROUTPUT_H_
#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>

#include "KinectAzure.h"
#include "KinectUtils.h"
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>
namespace traact::component::kinect {

class KinectAzureIROutput : public KinectAzureComponent {
 public:
    explicit KinectAzureIROutput(const std::string &name) : KinectAzureComponent(name) {

    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        traact::pattern::Pattern::Ptr
            pattern = getUncalibratedCameraPattern();
        pattern->name = "KinectAzureIROutput";

        std::set<std::string> frameRate = {"5", "15", "30"};
        pattern->addParameter("FrameRate", "30", frameRate);

        std::set<std::string>
            depthMode = {"OFF", "NFOV_2X2BINNED", "NFOV_UNBINNED", "WFOV_2X2BINNED", "WFOV_UNBINNED", "PASSIVE_IR"};
        pattern->addParameter("DepthMode", "NFOV_UNBINNED", depthMode);

        pattern->addParameter("SyncedImagesOnly", "true", pattern::CommonParameterEnums::bool_enum);

        pattern->addParameter("DepthDelayOffColor_usec", (int32_t) 0);

        std::set<std::string> syncModes = {"STANDALONE", "MASTER", "SUBORDINATE"};
        pattern->addParameter("HardwareSyncMode", "STANDALONE", syncModes);

        pattern->addParameter("SubordinateDelayOffMaster_usec", (int32_t) 0);

        pattern->addParameter("DisableStreamingIndicator", "false", pattern::CommonParameterEnums::bool_enum);

        return pattern;
    }

    bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override {
        std::shared_ptr<KinectAzureModule> k4a_module = std::dynamic_pointer_cast<KinectAzureModule>(module_);

        pattern::setValueFromParameter(parameter,
                                       "FrameRate",
                                       k4a_module->device_configuration.camera_fps,
                                       "30",
                                       KinectUtils::k4a_fps);
        pattern::setValueFromParameter(parameter,
                                       "DepthMode",
                                       k4a_module->device_configuration.depth_mode,
                                       "NFOV_UNBINNED",
                                       KinectUtils::k4a_depth_mode);
        pattern::setBoolValueFromParameter(parameter,
                                           "SyncedImagesOnly",
                                           k4a_module->device_configuration.synchronized_images_only,
                                           true);
        pattern::setValueFromParameter(parameter,
                                       "DepthDelayOffColor_usec",
                                       k4a_module->device_configuration.depth_delay_off_color_usec,
                                       0);
        pattern::setValueFromParameter(parameter,
                                       "HardwareSyncMode",
                                       k4a_module->device_configuration.wired_sync_mode,
                                       "STANDALONE",
                                       KinectUtils::k4a_wired_sync_mode);
        pattern::setValueFromParameter(parameter,
                                       "SubordinateDelayOffMaster_usec",
                                       k4a_module->device_configuration.subordinate_delay_off_master_usec,
                                       0);
        pattern::setBoolValueFromParameter(parameter,
                                           "DisableStreamingIndicator",
                                           k4a_module->device_configuration.disable_streaming_indicator,
                                           false);

        device_id_ = "Any";
        return true;
    }

    KinectOutputs GetOutputType() override {
        return KinectOutputs::IRImage;
    }

    void process(k4a::capture &capture, Timestamp ts) override {
        using namespace traact::vision;

        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (buffer) {
            const k4a::image inputImage = capture.get_ir_image();
            int w = inputImage.get_width_pixels();
            int h = inputImage.get_height_pixels();
            if (inputImage.get_format() == K4A_IMAGE_FORMAT_IR16) {
                cv::Mat image_buffer = cv::Mat(cv::Size(w, h),
                                               CV_16UC1,
                                               const_cast<void *>(static_cast<const void *>(inputImage.get_buffer())),
                                               cv::Mat::AUTO_STEP).clone();
                // copy image, would it be possible to take ownership of the data?
                //image_buffer = image_buffer.clone();
                auto &newData = buffer->getOutput<ImageHeader::NativeType, ImageHeader>(0);

                if (!newData.IsCpu() && !newData.IsGpu()) {
                    ImageHeader header;
                    header.width = w;
                    header.height = h;
                    header.opencv_matrix_type = image_buffer.type();
                    header.device_id = 0;
                    newData.init(header);
                }
                newData.SetCpuMat(image_buffer);
                /*if (newData.IsGpu())
                newData.GetGpuMat().upload(image_buffer);*/
            } else {
                SPDLOG_WARN("Received ir frame with unexpected format");
            }
        }

        SPDLOG_TRACE("commit depth");
        buffer->commit(true);
    }

};
}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::kinect::KinectAzureIROutput>("KinectAzureIROutput").constructor<std::string>()();
}

#endif //TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREIROUTPUT_H_
