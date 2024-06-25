/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>

#include "KinectAzure.h"
#include "KinectUtils.h"
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>
namespace traact::component::kinect {

class KinectAzureOutputDepth : public KinectAzureComponent {
 public:
    explicit KinectAzureOutputDepth(const std::string &name) : KinectAzureComponent(name) {

    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        traact::pattern::Pattern::Ptr
            pattern = getBasicCameraPattern();
        pattern->name = "traact::component::kinect::KinectAzureOutputDepth";
        return pattern;
    }

    KinectOutputs GetOutputType() override {
        return KinectOutputs::DepthImage;
    }

    k4a::image getImage(k4a::capture &capture) override {
        return capture.get_depth_image();
    }

};
CREATE_TRAACT_COMPONENT_FACTORY(KinectAzureOutputDepth)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
REGISTER_DEFAULT_COMPONENT(traact::component::kinect::KinectAzureOutputDepth)
END_TRAACT_PLUGIN_REGISTRATION
