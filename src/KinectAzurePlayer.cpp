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


#include "KinectAzurePlayer.h"
#include "KinectUtils.h"
#include <rttr/registration>

traact::TimestampType traact::component::vision::KinectAzurePlayer::GetFirstTimestamp() {
  return first_timestamp_;
}
bool traact::component::vision::KinectAzurePlayer::HasNext() {
  return !reached_end;//!frames.empty();
}
traact::TimestampType traact::component::vision::KinectAzurePlayer::GetNextTimestamp() {
  if(frames.empty()){
    SPDLOG_WARN("Buffer empty, could not load fast enough for realtime playback");
  }
  has_data_lock_.wait();
  return  frames.front().timestamp;


}
void traact::component::vision::KinectAzurePlayer::SendCurrent(traact::TimestampType ts) {
  using namespace traact::vision;


  //TODO need to check for valid buffer
  //if (domain_ts != TimestampType::min())

  auto buffer = request_callback_(ts);
  if(buffer == nullptr){
      SPDLOG_ERROR("could not get buffer");
      return;

  }


  frame_t current_frame = frames.front();
  frames.pop();

  frames_lock_.notify();

  /*
  if(!frames.try_pop(current_frame)) {
    SPDLOG_ERROR("trying to send current image with no data in queue");
    return;
  }
  */


  const k4a::image inputImage = current_frame.color_image;

  int w = inputImage.get_width_pixels();
  int h = inputImage.get_height_pixels();

  if (inputImage.get_format() == K4A_IMAGE_FORMAT_COLOR_BGRA32) {

    cv::Mat image_buffer = cv::Mat(cv::Size(w, h), CV_8UC4, const_cast<void*>(static_cast<const void*>(inputImage.get_buffer())), cv::Mat::AUTO_STEP);
    // copy image, would it be possible to take ownership of the data?
    //image_buffer = image_buffer.clone();
    auto &newData = buffer->getOutput<ImageHeader::NativeType, ImageHeader>(0);
    //auto &calibration = buffer.getOutput<CameraCalibrationHeader::NativeType, CameraCalibrationHeader>(1);
    //calibration = color_calibration_;

    if(!newData.IsCpu() && !newData.IsGpu()){
      ImageHeader header;
      header.width = w;
      header.height = h;
      header.opencv_matrix_type = image_buffer.type();
      header.device_id = 0;
      newData.init(header);
    }
    //newData.SetCpuMat(image_buffer);
      image_buffer.copyTo(newData.GetCpuMat());

    /*if (newData.IsCpu())
      image_buffer.copyTo(newData.GetCpuMat());
    if (newData.IsGpu())
      image_buffer.GetGpuMat().upload(image);*/
  }
  SPDLOG_TRACE("commit color");
    buffer->Commit(true);
}

bool traact::component::vision::KinectAzurePlayer::configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) {
    if(running_)
        return true;

    pattern::setValueFromParameter(parameter, "file", filename_, "/data/video.mkv");




  recording_.handle = k4a::playback::open(filename_.c_str());
  if (!recording_.handle) {

    SPDLOG_ERROR("{1}: could not open file: {0}", filename_, getName());
    return false;
  }
  SPDLOG_INFO("{1}: open file {0}", filename_, getName());
  recording_.handle.set_color_conversion(K4A_IMAGE_FORMAT_COLOR_BGRA32);
  recording_.record_config = recording_.handle.get_record_configuration();

  if (recording_.record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER) {


  } else if (recording_.record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_SUBORDINATE) {

  } else {

  }

  auto kinect_calib = recording_.handle.get_calibration();
  KinectUtils::k4a2traact(kinect_calib.color_camera_calibration, color_calibration_);
    KinectUtils::k4a2traact(kinect_calib.depth_camera_calibration, ir_calibration_);

  if (recording_.handle.get_recording_length().count() == 0) {
    //spdlog::error("PlaybackKinect4Azure[{0}] recording with no content: {1}", m_name, filename);
    return false;
  }

  if (!recording_.handle.get_next_capture(&(recording_.capture))) {
    //spdlog::error("PlaybackKinect4Azure[{0}] recording cannot read first frame: {1}", m_name, filename);
    return false;
  }

  // skip the first 5 frames from the recording as they typically contain chunk frames/timestamps
  uint64_t min_ts;
  //spdlog::info("PlaybackKinect4Azure[{0}] skipping 5 frames from start for: {1}", m_name, filename);
  for (int i = 0; i < 5; i++) {
    //get_minimum_timestamp_from_capture(recording_, min_ts);
    if (!recording_.handle.get_next_capture(&(recording_.capture))) {
      SPDLOG_ERROR("{1}: could not read frame while skipping frame {0}", i, getName());
      return false;
    }
  }


  switch (recording_.record_config.camera_fps) {
    case K4A_FRAMES_PER_SECOND_5:
      //recording_.time_per_frame = std::chrono::microseconds(std::micro::den / (std::micro::num * 5));
      break;
    case K4A_FRAMES_PER_SECOND_15:
      //recording_.time_per_frame = std::chrono::microseconds(std::micro::den / (std::micro::num * 15));
      break;
    case K4A_FRAMES_PER_SECOND_30:
      //recording_.time_per_frame = std::chrono::microseconds(std::micro::den / (std::micro::num * 30));
      break;
  }




  return true;//ModuleComponent::configure(parameter, data);
}
bool traact::component::vision::KinectAzurePlayer::start() {
    SPDLOG_DEBUG("{0}: Starting Kinect Player", getName());
    thread_ = std::make_shared<std::thread>([this]{
        running_ = true;
        thread_loop();
    });
  return true;//ModuleComponent::start();
}
bool traact::component::vision::KinectAzurePlayer::stop() {
    if(!running_)
        return true;
    running_ = false;
    thread_->join();
  return true;//ModuleComponent::Stop();
}
bool traact::component::vision::KinectAzurePlayer::teardown() {


  return true;//ModuleComponent::teardown();
}
traact::component::vision::KinectAzurePlayer::KinectAzurePlayer(const std::string &name) : PlayerBaseComponent(name), has_data_lock_(10,0), frames_lock_(10, 10) {}
//traact::component::vision::KinectAzurePlayer::KinectAzurePlayer(const std::string &name) : Component(name, ComponentType::AsyncSource), has_data_lock_(10,0), frames_lock_(10, 10) {}

void traact::component::vision::KinectAzurePlayer::thread_loop() {

  bool reached_end = false;
  while(running_ && !reached_end){
    SPDLOG_INFO("{0}:read next frame to buffer", getName());
    reached_end = !read_frame();

  }

}

bool traact::component::vision::KinectAzurePlayer::read_frame() {
  using namespace std::chrono;

  frames_lock_.wait();

  SPDLOG_INFO("read frame, buffer lock {0} data lock {1} queue {2}", frames_lock_.count(), has_data_lock_.count(), frames.size());

  if (!recording_.handle.get_next_capture(&(recording_.capture))) {
    reached_end = true;
    SPDLOG_ERROR("{0}: could not read frame", getName());
    return false;
  }
  frame_t new_frame;
  new_frame.color_image = recording_.capture.get_color_image();

  auto ts_ns = duration_cast<nanoseconds>(new_frame.color_image.get_device_timestamp());
    //auto ts_ns = duration_cast<nanoseconds>(new_frame.color_image.get_system_timestamp());
  new_frame.timestamp = TimestampType::min() + ts_ns + std::chrono::seconds(10);
  TimestampType  ts =new_frame.timestamp;
  //SPDLOG_TRACE("{0}: new timestamp {1}", getName(), new_frame.timestamp.time_since_epoch().count());
  frames.push(std::move(new_frame));
  has_data_lock_.notify();
    SendCurrent(ts);
  return true;
}

// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

    using namespace rttr;
    registration::class_<traact::component::vision::KinectAzurePlayer>("KinectAzurePlayer").constructor<std::string>()();
}
