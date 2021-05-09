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

#include <traact/component/Component.h>
#include <traact/buffer/ComponentBuffer.h>
#include <traact/vision.h>
#include <traact/component/vision/BasicVisionPattern.h>
#include <k4arecord/playback.hpp>
#include <k4a/k4a.h>
#include <tbb/concurrent_queue.h>
#include <traact/util/Semaphore.h>
#include <thread>
#include <queue>
#include <rttr/registration>
#include <traact/util/Logging.h>
#include <traact/pattern/ParameterUtils.h>
#include <traact/vision_datatypes.h>
#include "KinectUtils.h"
#include <mutex>
namespace traact::component::vision {
    class KinectAzureSingleFilePlayer : public Component {
    public:
        traact::pattern::Pattern::Ptr GetPattern() const override{
            traact::pattern::spatial::SpatialPattern::Ptr
                    pattern = getCameraPattern();
            pattern->name = "KinectAzureSingleFilePlayer";

            pattern->addProducerPort("output_ir", traact::vision::ImageHeader::MetaType);
            pattern->addProducerPort("output_calibration_ir", traact::vision::CameraCalibrationHeader::MetaType);



            pattern->addStringParameter("file", "/data/video.mkv")
                        .addParameter("stop_after_n_frames", -1l, -1l, std::numeric_limits<std::int64_t>::max());


            return pattern;
        }

        explicit KinectAzureSingleFilePlayer(std::string name): Component(std::move(name), ComponentType::AsyncSource), has_data_lock_(10,0), frames_lock_(10, 10) {}

        bool configure(const nlohmann::json &parameter, buffer::ComponentBufferConfig *data) override{
            if(running_)
                return true;

            pattern::setValueFromParameter(parameter, "file", filename_, "/data/video.mkv");
            pattern::setValueFromParameter(parameter, "stop_after_n_frames", stop_after_n_frames_, stop_after_n_frames_);



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
                SPDLOG_ERROR("{0} recording with no content: {1}", getName(), filename_);
                return false;
            }

            if (!recording_.handle.get_next_capture(&(recording_.capture))) {
                SPDLOG_ERROR("{0} recording cannot read first frame: {1}", getName(), filename_);
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

            return true;
        }
        bool start() override{
            SPDLOG_INFO("{0}: Starting Kinect Player", getName());
            running_ = true;
            thread_ = std::make_shared<std::thread>([this]{
                thread_loop();
            });
            thread_sender_ = std::make_shared<std::thread>([this]{
                thread_loop_sender();
            });
            return true;
        }
        bool stop() override{

            {
                std::unique_lock<std::timed_mutex> guard(mutex_,  std::defer_lock);
                if (!guard.try_lock_for(std::chrono::milliseconds(100))) {
                    SPDLOG_ERROR("Timeout for lock stop, return false");
                    return false;
                }

                if(!running_)
                    return true;
                running_ = false;
            }

            thread_->join();
            // if the sender thread is about to exit because the playback finished
            // calling setSourceFinished would lead the thread to wait for itself to finish, in that case don't wait as it will finish anyway
            //if(!thread_loop_end_)
                thread_sender_->join();

            return true;
        }
        bool teardown() override{
            recording_.handle.close();
            return true;
        }


    private:

        typedef struct {
            k4a::playback handle;
            k4a_record_configuration_t record_config;
            k4a::capture capture;
        } recording_t;

        typedef struct {
            TimestampType timestamp;
            k4a::image color_image;
            k4a::image ir_image;
        } frame_t;

        std::int64_t stop_after_n_frames_{-1};
        std::uint64_t current_frame_idx_{0};
        std::timed_mutex mutex_;
        bool running_{false};
        //bool thread_loop_end_{false};
        std::string filename_;
        TimestampType  first_timestamp_;

        recording_t recording_;
        std::shared_ptr<std::thread> thread_;
        std::shared_ptr<std::thread> thread_sender_;
        std::queue<frame_t> frames;
        bool reached_end{false};
        Semaphore frames_lock_;
        Semaphore has_data_lock_;
        TimeDurationType last_offset_{0};

        traact::vision::CameraCalibration color_calibration_;
        traact::vision::CameraCalibration ir_calibration_;

        bool IsRunning() {
            std::unique_lock<std::timed_mutex> guard(mutex_,  std::defer_lock);
            if (!guard.try_lock_for(std::chrono::milliseconds(10))) {
                SPDLOG_ERROR("Timeout for lock IsRunning, return false");
                return false;
            }
            return running_ && !reached_end;
        }

        bool IsSenderRunning()  {
            std::unique_lock<std::timed_mutex> guard(mutex_,  std::defer_lock);
            if (!guard.try_lock_for(std::chrono::milliseconds(10))) {
                SPDLOG_ERROR("Timeout for lock IsSenderRunning, return false");
                return false;
            }
            return  running_ && (has_data_lock_.count() > 0 || !reached_end);
        }


        void thread_loop() {

            while (IsRunning()) {
                SPDLOG_TRACE("{0}: read next frame to buffer", getName());
                reached_end = !read_frame();

            };
        }
        void thread_loop_sender(){
            while(IsSenderRunning()) {
                SPDLOG_TRACE("{0}: wait for data", getName());
                while(IsSenderRunning()){
                    if(has_data_lock_.wait()) {
                        break;
                    } else {
                        SPDLOG_TRACE("{0}: timeout when waiting to send new frame", getName());
                    };
                }

                if(!IsSenderRunning()){
                    break;
                }



                frame_t current_frame = frames.front();
                frames.pop();
                frames_lock_.notify();

                TimestampType  ts = current_frame.timestamp;

                SPDLOG_TRACE("{0}: send next frame to network {1}", getName(), ts.time_since_epoch().count());

                auto buffer = request_callback_(ts);
                if(buffer == nullptr){
                    SPDLOG_ERROR("component {0} ts {1} rejected", name_, ts.time_since_epoch().count());
                    continue;
                }

                auto &color_output = buffer->getOutput<traact::vision::ImageHeader::NativeType, traact::vision::ImageHeader>(0);
                auto &color_calib = buffer->getOutput<traact::vision::CameraCalibrationHeader::NativeType, traact::vision::CameraCalibrationHeader>(1);
                auto &ir_output = buffer->getOutput<traact::vision::ImageHeader::NativeType, traact::vision::ImageHeader>(2);
                auto &ir_calib = buffer->getOutput<traact::vision::CameraCalibrationHeader::NativeType, traact::vision::CameraCalibrationHeader>(3);
                color_calib = color_calibration_;
                ir_calib = ir_calibration_;

                setImage(current_frame.color_image, color_output);
                setImage(current_frame.ir_image, ir_output);


                SPDLOG_TRACE("{0}: commit color", getName());
                buffer->Commit(true);

                current_frame_idx_++;
                if(current_frame_idx_ > stop_after_n_frames_){
                    {
                        std::unique_lock<std::timed_mutex> guard(mutex_,  std::defer_lock);
                        running_ = false;
                    }
                    setSourceFinished();
                }
            }


            SPDLOG_INFO("{0}: playback end", getName());
//            {
//                std::unique_lock<std::timed_mutex> guard(mutex_,  std::defer_lock);
//                if (!guard.try_lock_for(std::chrono::milliseconds(10))) {
//                    SPDLOG_ERROR("Timeout for lock playback end");
//                }
//                thread_loop_end_ = true;
//            }
            if(running_)
                setSourceFinished();


        };

        void setImage(const k4a::image& image, traact::vision::Image& result_image){
            int w = image.get_width_pixels();
            int h = image.get_height_pixels();
            cv::Mat image_buffer;

            switch(image.get_format()){
                case K4A_IMAGE_FORMAT_COLOR_BGRA32:{
                    image_buffer  = cv::Mat(cv::Size(w, h), CV_8UC4, const_cast<void*>(static_cast<const void*>(image.get_buffer())), cv::Mat::AUTO_STEP);
                    break;
                }
                case K4A_IMAGE_FORMAT_IR16: {
                    image_buffer  = cv::Mat(cv::Size(w, h), CV_16UC1, const_cast<void*>(static_cast<const void*>(image.get_buffer())), cv::Mat::AUTO_STEP);
                    break;
                }
                default:
                    SPDLOG_ERROR("image format not supported");
            }
            if(!result_image.IsCpu() && !result_image.IsGpu()){
                traact::vision::ImageHeader header;
                header.width = w;
                header.height = h;
                header.opencv_matrix_type = image_buffer.type();
                header.device_id = 0;
                result_image.init(header);
            }
            //TODO try to avoid copy image, would it be possible to take ownership of the data?
            image_buffer.copyTo(result_image.GetCpuMat());
        }

        bool read_frame(){
            using namespace std::chrono;

            try{
                //frames_lock_.wait();
                while(IsRunning()){
                    if(frames_lock_.wait()) {
                        break;
                    } else {
                        SPDLOG_TRACE("{0}: timeout when waiting to get new frame buffer", getName());
                    };
                }

                if(!IsRunning())
                    return false;

                TimeDurationType ts_ns = TimeDurationType::min();
                frame_t new_frame;
                SPDLOG_TRACE("reading component {2} frame {0} has_data {1}", frames_lock_.count(), has_data_lock_.count(), name_);
                while(ts_ns == TimeDurationType::min()){
                    if (!recording_.handle.get_next_capture(&(recording_.capture))) {
                        reached_end = true;
                        SPDLOG_INFO("{0}: reached end of file", getName());
                        return false;
                    }

                    new_frame.color_image = recording_.capture.get_color_image();
                    new_frame.ir_image = recording_.capture.get_ir_image();
                    if(new_frame.color_image == nullptr || new_frame.ir_image == nullptr){
                        SPDLOG_WARN("read image is null, skip and continue with next image");
                        continue;
                    } else
                        ts_ns = duration_cast<nanoseconds>(new_frame.color_image.get_device_timestamp());

                    if(ts_ns <= last_offset_) {
                        ts_ns = TimeDurationType(0);
                        SPDLOG_WARN("timestamp not monotonic pref ts {0}, ts {1}, skip", last_offset_.count(), ts_ns.count());
                        continue;
                    }
                    last_offset_ = ts_ns;
                }

                new_frame.timestamp = TimestampType::min() + ts_ns;
                spdlog::info("{0}: new frame data with ts {1}", getName(), new_frame.timestamp.time_since_epoch().count());
                frames.push(std::move(new_frame));
                has_data_lock_.notify();
            } catch ( ... ) {
                SPDLOG_ERROR("exception trying to read frame");
                return false;
            }





            return true;
        };

        RTTR_ENABLE(Component)

    };

}



// It is not possible to place the macro multiple times in one cpp file. When you compile your plugin with the gcc toolchain,
// make sure you use the compiler option: -fno-gnu-unique. otherwise the unregistration will not work properly.
RTTR_PLUGIN_REGISTRATION // remark the different registration macro!
{

using namespace rttr;
registration::class_<traact::component::vision::KinectAzureSingleFilePlayer>("KinectAzureSingleFilePlayer").constructor<std::string>()();
}
