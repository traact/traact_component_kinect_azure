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

#ifndef TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_KINECTAZURE_H_
#define TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_KINECTAZURE_H_

#include <traact/traact.h>
#include <traact/vision.h>



#define WITH_BODYTRACKING

#include <k4a/k4a.hpp>
#ifdef WITH_BODYTRACKING
#include <k4abt.hpp>
#endif

#include <thread>

namespace traact::component::vision {

enum class KinectOutputs {
  ColorImage=0,
  DepthImage,
  IRImage,
  BodyTracking,
  PlayerMask,
  Count
};

class KinectAzureComponent : public ModuleComponent {
 public:
  KinectAzureComponent(std::string name)
      : ModuleComponent(name,
                        traact::component::ComponentType::AsyncSource, ModuleType::UniqueDataflowParameter) {
  };

  virtual void process(k4a::capture& capture, TimestampType ts);
  virtual void process(k4abt::frame& capture, TimestampType ts);
  virtual void noValidInput(TimestampType ts);
  virtual KinectOutputs GetOutputType() = 0;

  std::string GetModuleKey() override;
  Module::Ptr InstantiateModule() override;


 protected:
  std::string device_id_;
    RTTR_ENABLE(ModuleComponent)
};

class KinectAzureModule : public Module {
 public:
  KinectAzureModule();
  bool init(ComponentPtr module_component) override;
  bool start(ComponentPtr module_component) override;
  bool stop(ComponentPtr module_component) override;
  bool teardown(ComponentPtr module_component) override;

  k4a_device_configuration_t device_configuration{K4A_DEVICE_CONFIG_INIT_DISABLE_ALL};

#ifdef WITH_BODYTRACKING
  k4abt_tracker_configuration_t tracker_configuration;
#endif
 private:
  KinectAzureComponent* outputs_[static_cast<int>(KinectOutputs::Count)];

  std::shared_ptr<std::thread> thread_;
  bool initialized_{false};
  bool running_{false};
  std::mutex component_lock_;

  k4a::device device_;


#ifdef WITH_BODYTRACKING
  k4abt::tracker tracker_;
  bool bodytracking_enabled_{false};
#endif

  void threadLoop();
};



}

#endif //TRAACTMULTI_TRAACT_VISION_INCLUDE_TRAACT_COMPONENT_VISION_KINECTAZURE_H_
