/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZURECOLOROUTPUT_H_
#define TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZURECOLOROUTPUT_H_

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>

#include "KinectAzure.h"
#include "KinectUtils.h"
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>
namespace traact::component::kinect {

class KinectAzureColorOutput : public KinectAzureComponent {
 public:
    explicit KinectAzureColorOutput(const std::string &name) : KinectAzureComponent(name) {

    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        traact::pattern::Pattern::Ptr
            pattern = getUncalibratedCameraPattern();
        pattern->name = "KinectAzureColorOutput";

        std::set<std::string> colorResolution = {"OFF", "720P", "1080P", "1440P", "1536P", "2160P", "3072P"};
        pattern->addParameter("ColorResolution", "1536P", colorResolution);

        std::set<std::string> colorImageFormat = {"OFF", "COLOR_MJPG", "COLOR_NV12", "COLOR_YUY2", "COLOR_BGRA32"};
        pattern->addParameter("ColorImageFormat", "COLOR_BGRA32", colorImageFormat);

        std::set<std::string> frameRate = {"5", "15", "30"};
        pattern->addParameter("FrameRate", "30", frameRate);

        std::set<std::string> syncModes = {"STANDALONE", "MASTER", "SUBORDINATE"};
        pattern->addParameter("HardwareSyncMode", "STANDALONE", syncModes);

        pattern->addParameter("SubordinateDelayOffMaster_usec", (int32_t) 0);

        pattern->addParameter("DisableStreamingIndicator", "false", pattern::CommonParameterEnums::bool_enum);

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        std::shared_ptr<KinectAzureModule> k4a_module = std::dynamic_pointer_cast<KinectAzureModule>(module_);

        pattern::setValueFromParameter(pattern_instance,
                                       "ColorResolution",
                                       k4a_module->device_configuration.color_resolution,
                                       "1080P",
                                       KinectUtils::k4a_color_resolution);
        pattern::setValueFromParameter(pattern_instance,
                                       "ColorImageFormat",
                                       k4a_module->device_configuration.color_format,
                                       "COLOR_BGRA32",
                                       KinectUtils::k4a_image_format);
        pattern::setValueFromParameter(pattern_instance,
                                       "FrameRate",
                                       k4a_module->device_configuration.camera_fps,
                                       "30",
                                       KinectUtils::k4a_fps);
        pattern::setValueFromParameter(pattern_instance,
                                       "HardwareSyncMode",
                                       k4a_module->device_configuration.wired_sync_mode,
                                       "STANDALONE",
                                       KinectUtils::k4a_wired_sync_mode);
        pattern::setValueFromParameter(pattern_instance,
                                       "SubordinateDelayOffMaster_usec",
                                       k4a_module->device_configuration.subordinate_delay_off_master_usec,
                                       0);
        pattern::setBoolValueFromParameter(pattern_instance,
                                           "DisableStreamingIndicator",
                                           k4a_module->device_configuration.disable_streaming_indicator,
                                           false);

        device_id_ = "Any";
        return KinectAzureComponent::configure(pattern_instance, data);
    }

    KinectOutputs GetOutputType() override {
        return KinectOutputs::ColorImage;
    }

    void process(k4a::capture &capture, Timestamp ts) override {
        using namespace traact::vision;

        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (!buffer) {
            SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
            return;
        }

        const k4a::image inputImage = capture.get_color_image();

        int w = inputImage.get_width_pixels();
        int h = inputImage.get_height_pixels();

        if (inputImage.get_format() == K4A_IMAGE_FORMAT_COLOR_BGRA32) {

            cv::Mat image_buffer = cv::Mat(cv::Size(w, h),
                                           CV_8UC4,
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

            /*if (newData.IsCpu())
              image.copyTo(newData.GetCpuMat());
            if (newData.IsGpu())
              newData.GetGpuMat().upload(image);*/
        }

        SPDLOG_TRACE("commit color");
        buffer->commit(true);
    }



};
}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::kinect::KinectAzureColorOutput>("KinectAzureColorOutput").constructor<std::string>()();
}

#endif //TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZURECOLOROUTPUT_H_
