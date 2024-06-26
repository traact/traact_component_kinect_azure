/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_KINECTAZURE_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_KINECTAZURE_H_

#include <traact/traact.h>
#include <traact/vision.h>

#include <k4a/k4a.hpp>
#ifdef WITH_BODYTRACKING
#include <k4abt.hpp>
#endif

#include <thread>
#include <utility>

namespace traact::component::kinect {

enum class KinectOutputs {
    ColorImage = 0,
    DepthImage,
    IRImage,
    PlayerMask,
    BodyTracking,
  Calibration,
    Count
};

class KinectAzureComponent : public ModuleComponent {
 public:
    KinectAzureComponent(std::string name)
        : ModuleComponent(std::move(name), ModuleType::UNIQUE_DATAFLOW_PARAMETER) {
    };

    void setModuleKey(const pattern::instance::PatternInstance &pattern_instance) override;
    virtual k4a::image getImage(k4a::capture &capture);
    virtual void process(k4abt::frame &capture, Timestamp ts);
    virtual void process(const k4a::calibration &calibration, Timestamp ts);
    virtual void noValidInput(Timestamp ts);
    virtual KinectOutputs GetOutputType() = 0;

    std::string getModuleKey() override;
    Module::Ptr instantiateModule() override;

    void process_internal(k4a::capture &capture, Timestamp ts);


    static traact::pattern::Pattern::Ptr getBasicCameraPattern();

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override;

   protected:
    std::string device_id_;

};

class KinectAzureModule : public Module {
 public:
    KinectAzureModule();
    bool init(ComponentPtr module_component) override;
    bool start(ComponentPtr module_component) override;
    bool stop(ComponentPtr module_component) override;
    bool teardown(ComponentPtr module_component) override;

    k4a_device_configuration_t device_configuration{K4A_DEVICE_CONFIG_INIT_DISABLE_ALL};
    std::string device_id;

#ifdef WITH_BODYTRACKING
    k4abt_tracker_configuration_t tracker_configuration;
#endif
 private:
    KinectAzureComponent *outputs_[static_cast<int>(KinectOutputs::Count)];

    std::shared_ptr<std::thread> thread_;
    bool initialized_{false};
    bool running_{false};
    std::mutex component_lock_;

    k4a::device device_;
    k4a::calibration sensor_calibration_;
#ifdef WITH_BODYTRACKING
    k4abt::tracker tracker_;
    bool bodytracking_enabled_{false};
#endif

    void threadLoop();
};

}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_KINECTAZURE_H_
