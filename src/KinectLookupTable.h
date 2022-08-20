
#ifndef TRAACT_COMPONENT_KINECT_AZURE_SRC_KINECTLOOKUPTABLE_H_
#define TRAACT_COMPONENT_KINECT_AZURE_SRC_KINECTLOOKUPTABLE_H_

#include <traact/vision.h>

namespace traact::vision {

bool createXyLookupTable(const CameraCalibration& calib, cv::Mat& xy_table);
} // traact

#endif //TRAACT_COMPONENT_KINECT_AZURE_SRC_KINECTLOOKUPTABLE_H_
