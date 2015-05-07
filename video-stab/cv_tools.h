#ifndef CV_TOOLS_H
#define CV_TOOLS_H

#include "opencv2/core/core.hpp"

#include "VX/vx.h"
#include "vx_debug.h"

bool CV2VX(vx_image vxImage, cv::Mat cvImage);
bool VX2CV(vx_image vxImage, cv::Mat& cvImage);
cv::Mat MergeImage(cv::Mat up, cv::Mat down);

#endif // CV_TOOLS_H
