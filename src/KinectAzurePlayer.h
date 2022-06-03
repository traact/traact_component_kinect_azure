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

#ifndef TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREPLAYER_H_
#define TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREPLAYER_H_

#include <traact/component/PlayerBaseComponent.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>
#include <k4arecord/playback.hpp>
#include <k4a/k4a.h>
#include <tbb/concurrent_queue.h>
#include <traact/util/Semaphore.h>

namespace traact::component::vision {
class KinectAzurePlayer : public PlayerBaseComponent {
 public:
  traact::pattern::Pattern::Ptr GetPattern() const override{
    traact::pattern::Pattern::Ptr
        pattern = getCameraPattern();
    pattern->name = "KinectAzurePlayer";



    pattern->addStringParameter("file", "/data/video.mkv");


    return pattern;
  }

  explicit KinectAzurePlayer(const std::string &name);

  bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override;
  bool start() override;
  bool stop() override;
  bool teardown() override;

  TimestampType GetFirstTimestamp() override;
  TimestampType GetNextTimestamp() override;
  bool HasNext() override;
  void SendCurrent(TimestampType ts) override;


 private:

  typedef struct {
    k4a::playback handle;
    k4a_record_configuration_t record_config;
    k4a::capture capture;
  } recording_t;

  typedef struct {
    TimestampType timestamp;
    k4a::image color_image;
  } frame_t;

  bool running_{false};
  std::string filename_;
  TimestampType  first_timestamp_;

  recording_t recording_;
  std::shared_ptr<std::thread> thread_;
  std::queue<frame_t> frames;
  bool reached_end{false};
  Semaphore frames_lock_;
  Semaphore has_data_lock_;

  traact::vision::CameraCalibration color_calibration_;
  traact::vision::CameraCalibration ir_calibration_;


  void thread_loop();

  bool read_frame();

  RTTR_ENABLE(PlayerBaseComponent);

};

}

#endif //TRAACTMULTI_TRAACT_KINECT_AZURE_SRC_KINECTAZUREPLAYER_H_
