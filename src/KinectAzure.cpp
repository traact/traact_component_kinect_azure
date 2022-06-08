/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/



#include "KinectAzure.h"
#include "KinectUtils.h"

#include <tbb/task_group.h>

#include <traact/buffer/SourceComponentBuffer.h>

std::string traact::component::vision::KinectAzureComponent::getModuleKey() {
    return "KinectAzureDevice_" + device_id_;
}
traact::component::Module::Ptr traact::component::vision::KinectAzureComponent::instantiateModule() {
    SPDLOG_DEBUG("instantiate kinect device module");
    return std::make_shared<KinectAzureModule>();
}
void traact::component::vision::KinectAzureComponent::process(k4a::capture &capture, Timestamp ts) {

}
void traact::component::vision::KinectAzureComponent::process(k4abt::frame &capture, Timestamp ts) {

}
void traact::component::vision::KinectAzureComponent::noValidInput(traact::Timestamp ts) {
    SPDLOG_INFO("no valid input for ts: {0}", ts.time_since_epoch().count());
    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    buffer->commit(false);
}

bool traact::component::vision::KinectAzureModule::init(traact::component::Module::ComponentPtr module_component) {
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

    try {

        device_ = k4a::device::open(K4A_DEVICE_DEFAULT);
        initialized_ = true;
    } catch (k4a::error e) {
        SPDLOG_ERROR("Error trying to open Kinect Azure Device: {0}", e.what());
        return false;
    }

    return true;
}
bool traact::component::vision::KinectAzureModule::start(traact::component::Module::ComponentPtr module_component) {
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
bool traact::component::vision::KinectAzureModule::stop(traact::component::Module::ComponentPtr module_component) {
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
bool traact::component::vision::KinectAzureModule::teardown(traact::component::Module::ComponentPtr module_component) {
    std::lock_guard guard(component_lock_);
    if (!initialized_)
        return true;

    device_.close();
    return true;
}

void traact::component::vision::KinectAzureModule::threadLoop() {
    using namespace traact::vision;
    using namespace traact;

    int output_count = 0;

    try {
        tbb::task_group local_tasks;

        SPDLOG_INFO("open cameras");
        device_.start_cameras(&device_configuration);
        k4a::capture capture;

#ifdef WITH_BODYTRACKING
        if (bodytracking_enabled_) {
            SPDLOG_INFO("create body tracker");
            k4a::calibration sensor_calibration =
                device_.get_calibration(device_configuration.depth_mode, device_configuration.color_resolution);
            tracker_ = k4abt::tracker::create(sensor_calibration, tracker_configuration);
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

            // capture next image
            if (device_.get_capture(&capture, std::chrono::milliseconds(50))) {
                Timestamp toa_ts = now();

                //if(!capture.is_valid()) {
                //  SPDLOG_WARN("invalid capture");
                //  continue;
                //}
#ifdef WITH_BODYTRACKING
                if (bodytracking_enabled_) {
                    tracker_.enqueue_capture(capture);
                }
#endif

                // color, depth and ir image
                for (int i = 0; i < 3; ++i) {

                    if (outputs_[i]) {
                        outputs_[i]->process(capture, toa_ts);
                        //local_tasks.run([&, i]{
                        //  outputs_[i]->process(capture, toa_ts);
                        //});
                    }
                }

#ifdef WITH_BODYTRACKING
                if (bodytracking_enabled_) {

                    k4abt::frame body_frame = tracker_.pop_result(std::chrono::milliseconds(30));
                    if (body_frame != nullptr) {


                        // body tracking, player index mask
                        for (int i = 3; i < 5; ++i) {
                            if (outputs_[i]) {
                                outputs_[i]->process(body_frame, toa_ts);
                                //local_tasks.run([&, i]{
                                // outputs_[i]->process(body_frame, toa_ts);
                                //});
                            }
                        }

                    } else {
                        SPDLOG_WARN("Pop body frame result time out!");
                        for (int i = 3; i < 5; ++i) {
                            if (outputs_[i]) {
                                outputs_[i]->noValidInput(toa_ts);
                                //local_tasks.run([&, i]{
                                // outputs_[i]->process(body_frame, toa_ts);
                                //});
                            }
                        }
                    }
                }
#endif
                local_tasks.wait();

            }
        }

        device_.stop_cameras();
    } catch (const k4a::error &e) {
        SPDLOG_ERROR("Error in capture loop: {0}", e.what());
    }

    SPDLOG_TRACE("source quit loop");
    running_ = false;
}
traact::component::vision::KinectAzureModule::KinectAzureModule() {
    std::memset(&outputs_, 0, sizeof outputs_);
}
