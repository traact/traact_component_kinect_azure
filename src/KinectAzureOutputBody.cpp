/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/
#include <traact/traact.h>
#include <traact/vision.h>


#include "KinectAzure.h"
#include "KinectUtils.h"
#include <traact/spatialBody.h>
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>
namespace traact::component::kinect {

class KinectAzureOutputBody : public KinectAzureComponent {
 public:
    explicit KinectAzureOutputBody(const std::string &name) : KinectAzureComponent(name) {

    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        traact::pattern::Pattern::Ptr
            pattern = getBasicCameraPattern();
        pattern->name = "traact::component::kinect::KinectAzureOutputBody";

        return pattern;
    }

    bool configure(const pattern::instance::PatternInstance &pattern_instance, buffer::ComponentBufferConfig *data) override {
        std::shared_ptr<KinectAzureModule> k4a_module = std::dynamic_pointer_cast<KinectAzureModule>(module_);
        k4a_module->tracker_configuration.gpu_device_id = 0;
        k4a_module->tracker_configuration.processing_mode = K4ABT_TRACKER_PROCESSING_MODE_GPU;
        k4a_module->tracker_configuration.sensor_orientation = K4ABT_SENSOR_ORIENTATION_DEFAULT;
        return KinectAzureComponent::configure(pattern_instance, data);
    }

    KinectOutputs GetOutputType() override {
        return KinectOutputs::BodyTracking;
    }

    void process(k4abt::frame &body_frame, Timestamp ts) override {
        using namespace traact::spatial;

        auto buffer_future = request_callback_(ts);
        buffer_future.wait();
        auto buffer = buffer_future.get();
        if (!buffer) {
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
                traact::spatial::Translation3D pos(position.v[0] / 1000., -position.v[1] / 1000., -position.v[2] / 1000.);
                traact::spatial::Rotation3D rot(-orientation.v[0], -orientation.v[1], orientation.v[2], orientation.v[3]);
                bodyJoint.pose = pos * rot;
                bodyJoint.confidenceLevel = confidenceLevel;

                body.bodyJoints.emplace(static_cast<BodyJointType >(j), bodyJoint);
            }

            newData.emplace_back(std::move(body));
        }

        buffer->commit(true);
    }

};
CREATE_TRAACT_COMPONENT_FACTORY(KinectAzureOutputBody)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
REGISTER_DEFAULT_COMPONENT(traact::component::kinect::KinectAzureOutputBody)
END_TRAACT_PLUGIN_REGISTRATION

