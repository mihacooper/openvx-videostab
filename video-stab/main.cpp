#include <cstdio>
#include <iostream>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "vx_module.h"
#include "cv_tools.h"

#define MAX_PYRAMID_LEVELS 4

inline vx_int32 min(vx_int32 left, vx_int32 right)
{
    return left < right ? left : right;
}

inline vx_int32 max(vx_int32 left, vx_int32 right)
{
    return left > right ? left : right;
}

void InitParams(const int width, const int height, VideoStabParams& params)
{
    params.warp_gauss.scale = 0.85;
    params.warp_gauss.interpol = VX_INTERPOLATION_TYPE_BILINEAR;
    params.warp_gauss.gauss_size = 8;
    params.find_warp.fast_max_corners = 1000;
    params.find_warp.fast_thresh      = 50.f;

    params.find_warp.optflow_estimate = 0.01f;
    params.find_warp.optflow_max_iter = 30;
    params.find_warp.optflow_term     = VX_TERM_CRITERIA_BOTH;
    params.find_warp.optflow_wnd_size = 11;

    params.find_warp.pyramid_scale    = VX_SCALE_PYRAMID_HALF;
    params.find_warp.pyramid_level    = min(
            floor(log(vx_float32(params.find_warp.optflow_wnd_size) / vx_float32(width)) / log(params.find_warp.pyramid_scale)),
            floor(log(vx_float32(params.find_warp.optflow_wnd_size) / vx_float32(height)) / log(params.find_warp.pyramid_scale))
            );
    params.find_warp.pyramid_level = max(1, min(params.find_warp.pyramid_level, MAX_PYRAMID_LEVELS));
}

#define DEBUG_ZONES {VX_ZONE_ERROR}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("Use ./%s <input_video> <output_video>\n", argv[0]);
        return 0;
    }
    cv::VideoCapture cvReader(argv[1]); // video reader
    cv::VideoWriter  cvWriter;          // video writer
    VXVideoStab      vstub;             // stabilizator
    VideoStabParams  vs_params;         // params of stabilization

    int width  = cvReader.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cvReader.get(CV_CAP_PROP_FRAME_HEIGHT);
    int frames = cvReader.get(CV_CAP_PROP_FRAME_COUNT);
    /* Init video writer */
    if(!cvWriter.open(argv[2], CV_FOURCC('X', 'V', 'I', 'D'), cvReader.get(CV_CAP_PROP_FPS), cv::Size(width, height)))
    {
        std::cout << " Can't open output video file!" << std::endl;
        return 1;
    }
    /* Init parameters of stabilization */
    InitParams(width, height, vs_params);
    /* Build pipeline of stabilization */
    if(vstub.CreatePipeline(width, height, vs_params) != VX_SUCCESS)
        return 1;
    /* Enable debug zones */
    vstub.EnableDebug(DEBUG_ZONES);
    /**********************/

    cv::Mat cvImage;
    int counter = 0;
    while(true)
    {
        cvReader >> cvImage;
        if(cvImage.empty())
        {
            printf("End of video!\n");
            break;
        }
        vx_image vxImage = vstub.NewImage();
        if(!CV2VX(vxImage, cvImage)) break;
        vx_image out = vstub.Calculate();
        if(out)
        {
            if(!VX2CV(out, cvImage)) break;
            cvWriter << cvImage;
        }
        counter++;
        std::cout << counter << " from " << frames <<" processed frames" << std::endl;
    }
    cvWriter.release();
    vstub.DisableDebug(DEBUG_ZONES);
    return 0;
}
