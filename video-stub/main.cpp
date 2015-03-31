#include <cstdio>
#include <iostream>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "vx_module.h"
#include "cv_tools.h"

#define WINDOW_NAME "VideoStub"
#define MAX_PYRAMID_LEVELS 5

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
    params.gauss_size = 8;
    params.fast_max_corners = 2000;
    params.fast_thresh      = 50.f;

    params.optflow_estimate = 0.01f;
    params.optflow_max_iter = 100;
    params.optflow_term     = VX_TERM_CRITERIA_EPSILON;
    params.optflow_wnd_size = 21;

    params.pyramid_scale    = VX_SCALE_PYRAMID_HALF;
    params.pyramid_level    = min(
            floor(log(vx_float32(params.optflow_wnd_size) / vx_float32(width)) / log(params.pyramid_scale)),
            floor(log(vx_float32(params.optflow_wnd_size) / vx_float32(height)) / log(params.pyramid_scale))
            );
    params.pyramid_level = max(1, min(params.pyramid_level, MAX_PYRAMID_LEVELS));
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("Use ./%s <input_video> <output_video>\n", argv[0]);
        return 0;
    }

    cv::VideoCapture cvReader(argv[1]);
    cv::VideoWriter  cvWriter;

    VXVideoStab vstub;
    vstub.EnableDebug({VX_ZONE_ERROR/*, VX_ZONE_LOG, VX_ZONE_DELAY, VX_ZONE_IMAGE*/});
    VideoStabParams vs_params;

    cv::Mat cvImage, resImg;
    bool first = true;
    int counter = 0;
    while(true)
    {
        cvReader >> cvImage;
        cvImage.copyTo(resImg);
        if(cvImage.empty())
        {
            printf("End of video!\n");
            break;
        }
        if(first)
        {
            InitParams(cvImage.cols, cvImage.rows, vs_params);
            if(vstub.CreatePipeline(cvImage.cols, cvImage.rows, vs_params) != VX_SUCCESS)
                break;
            if(!cvWriter.open(argv[2], cvReader.get(CV_CAP_PROP_FOURCC), cvReader.get(CV_CAP_PROP_FPS), cv::Size(cvImage.cols, cvImage.rows * 2)))
            {
                std::cout << " Can't open output video file!" << std::endl;
                break;
            }
            first = false;
        }
        vx_image vxImage = vstub.NewImage();
        if(!CV2VX(vxImage, resImg))
        {
            printf("Can't convert image CV->VX. Stop!\n");
            break;
        }
        vx_image out = vstub.Calculate();
        if(out)
        {
            if(!VX2CV(out, resImg))
            {
                printf("Can't convert image VX->CV. Stop!\n");
                break;
            }
        }
        cv::Mat mergedImg = MergeImage(resImg, cvImage);
        //cv::imshow(WINDOW_NAME, mergedImg);
        //cv::waitKey(5);
        cvWriter << mergedImg;
        counter++;
        if(counter == 30){ break;}
        std::cout << counter << " processed frames" << std::endl;
    }
    cvWriter.release();
    printf("**** Performance ****\n");
    vstub.PrintPerf();
    printf("*********************\n");
    return 0;
}
