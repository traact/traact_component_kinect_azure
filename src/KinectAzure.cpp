/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/



#include "KinectAzure.h"
#include "KinectUtils.h"

#include <traact/buffer/SourceComponentBuffer.h>
#include <traact/component/vision/BasicVisionPattern.h>

class DeviceManager
{
 public:
  DeviceManager(DeviceManager const&) = delete;
  DeviceManager& operator=(DeviceManager const&) = delete;
  ~DeviceManager() = default;

  static DeviceManager* getInstance()
  {
    static DeviceManager instance{};
    return &instance;
  }

  [[nodiscard]] std::optional<uint32_t> findDevice(const std::string &serial) const{
    auto result = _devices.find(serial);
    if(result != _devices.cend()){
      return {result->second};
    } else {
      return {};
    }
  }

 private:
  explicit DeviceManager()  {
    uint32_t device_count = k4a_device_get_installed_count();

    if (device_count == 0)
    {
      spdlog::error("No Kinect Devices connected");
      return;
    }

    _devices.emplace("ANY", K4A_DEVICE_DEFAULT);

    for (uint32_t i = 0; i < device_count; i++)
    {

      k4a::device device = k4a::device::open(i);
      spdlog::debug("Found Kinect Device {0} Serial {1}", i , device.get_serialnum());
      _devices.emplace(device.get_serialnum(), i);
      device.close();
    }
  }
  std::map<std::string, uint32_t> _devices;

};

std::string traact::component::kinect::KinectAzureComponent::getModuleKey() {
    return "KinectAzureDevice_" + device_id_;
}
traact::component::Module::Ptr traact::component::kinect::KinectAzureComponent::instantiateModule() {
    SPDLOG_DEBUG("instantiate kinect device module");
    return std::make_shared<KinectAzureModule>();
}
k4a::image traact::component::kinect::KinectAzureComponent::getImage(k4a::capture &capture) {
  return {};
}
void traact::component::kinect::KinectAzureComponent::process(k4abt::frame &capture, Timestamp ts) {

}
void traact::component::kinect::KinectAzureComponent::noValidInput(traact::Timestamp ts) {
    SPDLOG_INFO("no valid input for ts: {0}", ts.time_since_epoch().count());
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    buffer->commit(false);
}
void traact::component::kinect::KinectAzureComponent::process_internal(k4a::capture &capture, traact::Timestamp ts) {
    using namespace traact::vision;
    const k4a::image inputImage = getImage(capture);

    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();

    if (!buffer) {
      SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
      return;
    }

    if(!buffer->requestUse()){
      return;
    }



    int w = inputImage.get_width_pixels();
    int h = inputImage.get_height_pixels();
    int cv_type = 0;

    using ImagePort = buffer::PortConfig<vision::ImageHeader, 0>;
    auto &newData = buffer->getOutput<ImagePort>();
    auto& header = buffer->getOutputHeader<ImagePort>();

    switch (inputImage.get_format()) {
      case K4A_IMAGE_FORMAT_COLOR_BGRA32:{
        cv_type = CV_8UC4;
        header.base_type = BaseType::UINT_8;
        header.pixel_format = PixelFormat::BGRA;
        break;
      }
      case K4A_IMAGE_FORMAT_DEPTH16:{
        cv_type = CV_16UC1;
        header.base_type = BaseType::UINT_16;
        header.pixel_format = PixelFormat::DEPTH;
        break;
      }
      case K4A_IMAGE_FORMAT_IR16:{
        cv_type = CV_16UC1;
        header.base_type = BaseType::UINT_16;
        header.pixel_format = PixelFormat::LUMINANCE;
        break;
      }
      default:{
        spdlog::error("unknown image format for kinect image capture: {0}", inputImage.get_format());
        return;
      }
    }

    cv::Mat image_buffer = cv::Mat(cv::Size(w, h),
                                   cv_type,
                                   const_cast<void *>(static_cast<const void *>(inputImage.get_buffer())),
                                   cv::Mat::AUTO_STEP).clone();

    newData.update(image_buffer, std::make_shared<k4a::image>(inputImage));


    header.setFrom(image_buffer);


    SPDLOG_TRACE("commit image");
    buffer->commit(true);
}

traact::pattern::Pattern::Ptr traact::component::kinect::KinectAzureComponent::getBasicCameraPattern() {
    traact::pattern::Pattern::Ptr
        pattern =
            std::make_shared<traact::pattern::Pattern>("KinectCameraPattern", Concurrency::SERIAL,ComponentType::ASYNC_SOURCE);

    pattern->addProducerPort("output", traact::vision::ImageHeader::NativeTypeName);

    pattern->addCoordinateSystem("ImagePlane")
            .addCoordinateSystem("Image", true)
            .addEdge("ImagePlane", "Image", "output");

    std::set<std::string> colorResolution = {"OFF", "720P", "1080P", "1440P", "1536P", "2160P", "3072P"};
    pattern->addParameter("ColorResolution", "1080P", colorResolution);

    std::set<std::string> colorImageFormat = {"OFF", "COLOR_MJPG", "COLOR_NV12", "COLOR_YUY2", "COLOR_BGRA32"};
    pattern->addParameter("ColorImageFormat", "COLOR_BGRA32", colorImageFormat);

    std::set<std::string> frameRate = {"5", "15", "30"};
    pattern->addParameter("FrameRate", "COLOR_BGRA32", frameRate);

    std::set<std::string>
        depthMode = {"OFF", "NFOV_2X2BINNED", "NFOV_UNBINNED", "WFOV_2X2BINNED", "WFOV_UNBINNED", "PASSIVE_IR"};
    pattern->addParameter("DepthMode", "NFOV_UNBINNED", depthMode);

    pattern->addParameter("SyncedImagesOnly", "true", pattern::CommonParameterEnums::bool_enum);

    pattern->addParameter("DepthDelayOffColor_usec", (int32_t) 0);

    std::set<std::string> syncModes = {"STANDALONE", "MASTER", "SUBORDINATE"};
    pattern->addParameter("HardwareSyncMode", "STANDALONE", syncModes);

    pattern->addParameter("SubordinateDelayOffMaster_usec", (int32_t) 0);

    pattern->addParameter("DisableStreamingIndicator", "false", pattern::CommonParameterEnums::bool_enum);

    pattern->addStringParameter("DeviceID", "ANY");

    return pattern;
}

bool traact::component::kinect::KinectAzureComponent::configure(const traact::pattern::instance::PatternInstance &pattern_instance, traact::buffer::ComponentBufferConfig *data) {
    std::shared_ptr<KinectAzureModule> k4a_module = std::dynamic_pointer_cast<KinectAzureModule>(module_);

    /*
    pattern_instance.setValueFromParameter("ColorResolution",k4a_module->device_configuration.color_resolution);
    pattern_instance.setValueFromParameter("ColorImageFormat",k4a_module->device_configuration.color_format);
    pattern_instance.setValueFromParameter("FrameRate", k4a_module->device_configuration.camera_fps);
    pattern_instance.setValueFromParameter("HardwareSyncMode",k4a_module->device_configuration.wired_sync_mode);
    pattern_instance.setValueFromParameter("SubordinateDelayOffMaster_usec",k4a_module->device_configuration.subordinate_delay_off_master_usec);
    pattern_instance.setValueFromParameter("DisableStreamingIndicator", k4a_module->device_configuration.disable_streaming_indicator);
     */
    k4a_module->device_configuration.color_resolution = k4a_color_resolution_t::K4A_COLOR_RESOLUTION_1536P;
    k4a_module->device_configuration.color_format = k4a_image_format_t::K4A_IMAGE_FORMAT_COLOR_BGRA32;
    k4a_module->device_configuration.camera_fps = k4a_fps_t::K4A_FRAMES_PER_SECOND_15;
    k4a_module->device_configuration.depth_mode = k4a_depth_mode_t::K4A_DEPTH_MODE_NFOV_UNBINNED;
    k4a_module->device_configuration.synchronized_images_only = true;


    std::string sync_mode;
    pattern_instance.setValueFromParameter("HardwareSyncMode",sync_mode);
    if(sync_mode == "MASTER"){
      k4a_module->device_configuration.wired_sync_mode = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_MASTER;
      k4a_module->device_configuration.subordinate_delay_off_master_usec = 0;
    } else {
      k4a_module->device_configuration.wired_sync_mode = k4a_wired_sync_mode_t::K4A_WIRED_SYNC_MODE_SUBORDINATE;
      k4a_module->device_configuration.subordinate_delay_off_master_usec = 100;
    }






    k4a_module->device_id = device_id_;

    k4a_module->init(this);
    return true;
}

void traact::component::kinect::KinectAzureComponent::process(const k4a::calibration &calibration, Timestamp ts) {
}
void traact::component::kinect::KinectAzureComponent::setModuleKey(const traact::pattern::instance::PatternInstance &pattern_instance) {
  pattern_instance.setValueFromParameter("DeviceID", device_id_);
  if(device_id_.empty()){
    SPDLOG_ERROR("Kinect DeviceID empty");
    return;
  } else {
    SPDLOG_INFO("Azure Kinect Module {0}", device_id_);
  }
}

bool traact::component::kinect::KinectAzureModule::init(traact::component::Module::ComponentPtr module_component) {
    std::lock_guard guard(component_lock_);

    auto k4a_component = dynamic_cast<KinectAzureComponent *>(module_component);
    outputs_[static_cast<int>(k4a_component->GetOutputType())] = k4a_component;


#ifdef WITH_BODYTRACKING
    switch (k4a_component->GetOutputType()) {
        case KinectOutputs::BodyTracking:
        case KinectOutputs::PlayerMask: {
            bodytracking_enabled_ = true;
            break;
        }
        default:break;
    }
#endif

    if (initialized_)
        return true;
    initialized_ = true;
    SPDLOG_INFO("Init Kinect Module");
    try {
        auto index = DeviceManager::getInstance()->findDevice(device_id);
        if(index) {
          SPDLOG_INFO("Open Kinect Device {0}, ID {1}", index.value(), device_id);
            device_ = k4a::device::open(index.value());
            sensor_calibration_ = device_.get_calibration(device_configuration.depth_mode, device_configuration.color_resolution);
        } else {
            SPDLOG_ERROR("Open Kinect Device failed: {0}", device_id);
        }

    } catch (k4a::error &e) {
        SPDLOG_ERROR("Error trying to open Kinect Azure Device: {0}", e.what());
        return false;
    }

    return true;
}
bool traact::component::kinect::KinectAzureModule::start(traact::component::Module::ComponentPtr module_component) {
    std::lock_guard guard(component_lock_);
    if (running_) {
        return true;
    }

    SPDLOG_DEBUG("Starting K4A Camera");
    running_ = true;



    thread_ = std::make_shared<std::thread>([this] {
        threadLoop();
    });
    return true;
}
bool traact::component::kinect::KinectAzureModule::stop(traact::component::Module::ComponentPtr module_component) {
    std::lock_guard guard(component_lock_);
    if (!running_) {
        return true;
    }
    SPDLOG_DEBUG("Stopping K4A Camera");
    if (thread_) {
        running_ = false;
        thread_->join();
        return true;
    }
    return false;
}
bool traact::component::kinect::KinectAzureModule::teardown(traact::component::Module::ComponentPtr module_component) {
    std::lock_guard guard(component_lock_);
    if (!initialized_)
        return true;

    device_.close();
    return true;
}

void traact::component::kinect::KinectAzureModule::threadLoop() {
    using namespace traact::vision;
    using namespace traact;

    int output_count = 0;

    try {
        //tbb::task_group local_tasks;
        SPDLOG_INFO("open cameras");
        device_.start_cameras(&device_configuration);


        k4a::capture capture;

#ifdef WITH_BODYTRACKING
        if (bodytracking_enabled_) {
            SPDLOG_INFO("create body tracker");
            tracker_ = k4abt::tracker::create(sensor_calibration_, tracker_configuration);
        }
#endif

        for (;;) {
            if (!running_) {
                break;
            }

            /*
             * note on K4A's image.get_system_timestamp()
             *
             *     Returns the system timestamp of the image. Timestamps are recorded by the host.
             *     They may be used for relative comparision, as they are relative to the corresponding system clock.
             *     The absolute value is a monotonic count from an arbitrary point in the past.
             *     The system timestamp is captured at the moment host PC finishes receiving the image.
             *     On Linux the system timestamp is read from clock_gettime(CLOCK_MONOTONIC), which measures realtime
             *     and is not impacted by adjustments to the system clock. It starts from an arbitrary point in the past.
             *     On Windows the system timestamp is read from QueryPerformanceCounter(), it also measures realtime and
             *     is not impacted by adjustments to the system clock. It also starts from an arbitrary point in the past.
             */

            /*
             * note on K4A's image.get_device_timestamp()
             *
             *     Returns the device timestamp of the image, as captured by the hardware. Timestamps are recorded by the
             *     device and represent the mid-point of exposure. They may be used for relative comparison,
             *     but their absolute value has no defined meaning.
             */

            // skip first 10 captures
            for (int i = 0; i < 10; ++i) {
              device_.get_capture(&capture, std::chrono::milliseconds(50));
            }

            // capture next image
            if (device_.get_capture(&capture, std::chrono::milliseconds(50))) {

                Timestamp toa_ts = now();
                SPDLOG_INFO("new image {0}, ts {1}", device_id, toa_ts);

                if(!capture.is_valid()) {
                  SPDLOG_WARN("invalid capture");
                  continue;
                }
#ifdef WITH_BODYTRACKING
                if (bodytracking_enabled_) {
                    tracker_.enqueue_capture(capture);
                }
#endif

                // color, depth and ir image
                for (int i = 0; i < 3; ++i) {

                    if (outputs_[i]) {
                      outputs_[i]->process_internal(capture, toa_ts);
                    }
                }

                if (outputs_[static_cast<int>(KinectOutputs::Calibration)]) {
                    outputs_[static_cast<int>(KinectOutputs::Calibration)]->process(sensor_calibration_, toa_ts);
                }

#ifdef WITH_BODYTRACKING
                if (bodytracking_enabled_) {

                    k4abt::frame body_frame = tracker_.pop_result(std::chrono::milliseconds(30));
                    if (body_frame != nullptr) {


                        // body tracking, player index mask
                        for (int i = 3; i < 5; ++i) {
                            if (outputs_[i]) {
                                outputs_[i]->process(body_frame, toa_ts);
                            }
                        }

                    } else {
                        SPDLOG_WARN("Pop body frame result time out!");
                        for (int i = 3; i < 5; ++i) {
                            if (outputs_[i]) {
                                outputs_[i]->noValidInput(toa_ts);
                            }
                        }
                    }
                }
#endif
            }
        }

        device_.stop_cameras();
    } catch (const k4a::error &e) {
        SPDLOG_ERROR("Error in capture loop: {0}", e.what());
    }

    SPDLOG_TRACE("source quit loop");
    running_ = false;
}

traact::component::kinect::KinectAzureModule::KinectAzureModule() {
    std::memset(&outputs_, 0, sizeof outputs_);
}
