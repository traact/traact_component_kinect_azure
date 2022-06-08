/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

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
    traact::pattern::Pattern::Ptr GetPattern() const override {
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

    Timestamp getFirstTimestamp() override;
    Timestamp getNextTimestamp() override;
    bool hasNext() override;
    void sendCurrent(Timestamp ts) override;

 private:

    typedef struct {
        k4a::playback handle;
        k4a_record_configuration_t record_config;
        k4a::capture capture;
    } recording_t;

    typedef struct {
        Timestamp timestamp;
        k4a::image color_image;
    } frame_t;

    bool running_{false};
    std::string filename_;
    Timestamp first_timestamp_;

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
