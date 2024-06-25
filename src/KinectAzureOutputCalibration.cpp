/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/

#include <traact/traact.h>
#include <traact/vision.h>
#include "KinectAzure.h"
#include "KinectUtils.h"
#include <rttr/registration>
#include <traact/buffer/SourceComponentBuffer.h>
#include "traact/util/KinectLookupTable.h"
#include <traact/opencv/OpenCVUtils.h>

namespace traact::component::kinect {

class KinectAzureOutputCalibration : public KinectAzureComponent {
 public:
  using OutPort_ColorCalibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 0>;
  using OutPort_IrCalibration = buffer::PortConfig<traact::vision::CameraCalibrationHeader, 1>;
  using OutPort_XYTable = buffer::PortConfig<traact::vision::ImageHeader, 2>;
  using OutPort_DepthToColor = buffer::PortConfig<traact::spatial::Pose6DHeader, 3>;

  explicit KinectAzureOutputCalibration(const std::string &name) : KinectAzureComponent(name) {

  }

  static traact::pattern::Pattern::Ptr GetPattern() {

    traact::pattern::Pattern::Ptr
        pattern =
            std::make_shared<traact::pattern::Pattern>("traact::component::kinect::KinectAzureOutputCalibration", Concurrency::SERIAL,ComponentType::ASYNC_SOURCE);

    pattern->addProducerPort<OutPort_ColorCalibration>("output_calibration")
            .addProducerPort<OutPort_IrCalibration>("output_ir_calibration")
            .addProducerPort<OutPort_XYTable>("output_xy_table")
            .addProducerPort<OutPort_DepthToColor>("output_color_to_depth");


    pattern->addCoordinateSystem("Camera_Color")
        .addCoordinateSystem("ImagePlane_Color")
        .addCoordinateSystem("Camera_Depth")
        .addCoordinateSystem("ImagePlane_Depth")
        .addEdge("ImagePlane", "Image", "output_calibration");

    std::set<std::string> colorResolution = {"OFF", "720P", "1080P", "1440P", "1536P", "2160P", "3072P"};
    pattern->addParameter("ColorResolution", "1080P", colorResolution);

    std::set<std::string> colorImageFormat = {"OFF", "COLOR_MJPG", "COLOR_NV12", "COLOR_YUY2", "COLOR_BGRA32"};
    pattern->addParameter("ColorImageFormat", "COLOR_BGRA32", colorImageFormat);

    std::set<std::string> frameRate = {"5", "15", "30"};
    pattern->addParameter("FrameRate", "COLOR_BGRA32", frameRate);

    std::set<std::string>
        depthMode = {"OFF", "NFOV_2X2BINNED", "NFOV_UNBINNED", "WFOV_2X2BINNED", "WFOV_UNBINNED", "PASSIVE_IR"};
    pattern->addParameter("DepthMode", "NFOV_UNBINNED", depthMode);

    pattern->addParameter("SyncedImagesOnly", "true", pattern::CommonParameterEnums::bool_enum);

    pattern->addParameter("DepthDelayOffColor_usec", (int32_t) 0);

    std::set<std::string> syncModes = {"STANDALONE", "MASTER", "SUBORDINATE"};
    pattern->addParameter("HardwareSyncMode", "STANDALONE", syncModes);

    pattern->addParameter("SubordinateDelayOffMaster_usec", (int32_t) 0);

    pattern->addParameter("DisableStreamingIndicator", "false", pattern::CommonParameterEnums::bool_enum);

    pattern->addStringParameter("CameraID", "ANY");

    return pattern;
  }



  KinectOutputs GetOutputType() override {
    return KinectOutputs::Calibration;
  }

  void process(const k4a::calibration &calibration, Timestamp ts) override {

    auto buffer_future = request_callback_(ts);
    buffer_future.wait();
    auto buffer = buffer_future.get();
    if (!buffer) {
      SPDLOG_WARN("Could not get source buffer for ts {0}", ts.time_since_epoch().count());
      return;
    }

    if(!initialized_){
      KinectUtils::k4a2traact(calibration.color_camera_calibration, color_calibration_);
      KinectUtils::k4a2traact(calibration.depth_camera_calibration, ir_calibration_);
      xy_table_ = cv::Mat(cv::Size(ir_calibration_.width, ir_calibration_.height), CV_32FC2);
      vision::createXyLookupTable(ir_calibration_, xy_table_);

      auto color_to_depth = calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR];
      auto r_vec = cv::Mat(3, 3, CV_32FC1, static_cast<float *>( color_to_depth.rotation));
      auto t_vec = cv::Mat(3, 1, CV_32FC1, static_cast<float *>(color_to_depth.translation));
      Eigen::Matrix3<traact::Scalar> r_eigen;
      cv2eigen(r_vec, r_eigen);
      Eigen::Vector3<traact::Scalar> t_eigen(t_vec.at<float>(0)/1000.0f, t_vec.at<float>(1)/1000.0f, t_vec.at<float>(2)/1000.0f);

      Eigen::Matrix4<traact::Scalar> tmp_pose;
      tmp_pose.setIdentity();
      tmp_pose.block<3, 3>(0, 0) = r_eigen;
      tmp_pose.block<3, 1>(0, 3) = t_eigen;



      color_to_depth_ = spatial::Pose6D(tmp_pose);

      initialized_ = true;
    }

    {
      auto &output = buffer->getOutput<OutPort_ColorCalibration>();
      output = color_calibration_;
    }

    {
      auto &output = buffer->getOutput<OutPort_IrCalibration >();
      output = ir_calibration_;
    }

    {
      auto &output = buffer->getOutput<OutPort_XYTable>();
      auto &header = buffer->getOutputHeader<OutPort_XYTable>();
      header.pixel_format = vision::PixelFormat::UV2;
      header.base_type = BaseType::FLOAT_32;
      header.channels = 2;
      header.setFrom(xy_table_);
      output.update(xy_table_);
    }

    {
      buffer->getOutput<OutPort_DepthToColor>() = color_to_depth_;
    }







    buffer->commit(true);
  }

 private:
  traact::vision::CameraCalibration color_calibration_;
  traact::vision::CameraCalibration ir_calibration_;
  cv::Mat xy_table_;
  spatial::Pose6D color_to_depth_;
  bool initialized_{false};
};
CREATE_TRAACT_COMPONENT_FACTORY(KinectAzureOutputCalibration)
}

BEGIN_TRAACT_PLUGIN_REGISTRATION
REGISTER_DEFAULT_COMPONENT(traact::component::kinect::KinectAzureOutputCalibration)
END_TRAACT_PLUGIN_REGISTRATION