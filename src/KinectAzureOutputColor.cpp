/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "KinectAzure.h"
#include "KinectUtils.h"
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>

namespace traact::component::kinect {

class KinectAzureOutputColor : public KinectAzureComponent {
 public:
    explicit KinectAzureOutputColor(const std::string &name) : KinectAzureComponent(name) {

    }

    static traact::pattern::Pattern::Ptr GetPattern() {
        traact::pattern::Pattern::Ptr
            pattern = getBasicCameraPattern();
        pattern->name = "traact::component::kinect::KinectAzureOutputColor";

        return pattern;
    }

    KinectOutputs GetOutputType() override {
        return KinectOutputs::ColorImage;
    }

    k4a::image getImage(k4a::capture &capture) override {
        return capture.get_color_image();
    }



};
CREATE_TRAACT_COMPONENT_FACTORY(KinectAzureOutputColor)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
REGISTER_DEFAULT_COMPONENT(traact::component::kinect::KinectAzureOutputColor)
END_TRAACT_PLUGIN_REGISTRATION