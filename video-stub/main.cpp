#include <cstdio>
#include <iostream>
#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "vx_module.h"
#include "cv_tools.h"

#define MAX_PYRAMID_LEVELS 3

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
    params.warp_gauss.interpol = VX_INTERPOLATION_TYPE_BILINEAR;
    params.warp_gauss.gauss_size = 8;
    params.find_warp.fast_max_corners = 500;
    params.find_warp.fast_thresh      = 50.f;

    params.find_warp.optflow_estimate = 0.01f;
    params.find_warp.optflow_max_iter = 30;
    params.find_warp.optflow_term     = VX_TERM_CRITERIA_BOTH;
    params.find_warp.optflow_wnd_size = 21;

    params.find_warp.pyramid_scale    = VX_SCALE_PYRAMID_ORB;
    params.find_warp.pyramid_level    = min(
            floor(log(vx_float32(params.find_warp.optflow_wnd_size) / vx_float32(width)) / log(params.find_warp.pyramid_scale)),
            floor(log(vx_float32(params.find_warp.optflow_wnd_size) / vx_float32(height)) / log(params.find_warp.pyramid_scale))
            );
    params.find_warp.pyramid_level = max(1, min(params.find_warp.pyramid_level, MAX_PYRAMID_LEVELS));
}

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("Use ./%s <input_video> <output_video>\n", argv[0]);
        return 0;
    }
    std::vector<vx_image> vxImages;
    vxImages.reserve(500);
    cv::VideoCapture cvReader(argv[1]);
    cv::VideoWriter  cvWriter;

    VXVideoStab vstub;
    vstub.EnableDebug({VX_ZONE_ERROR});
    VideoStabParams vs_params;
    int width, height;
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
            width = cvImage.cols;
            height = cvImage.rows;
            InitParams(width, height, vs_params);
            if(vstub.CreatePipeline(width, height, vs_params) != VX_SUCCESS)
                break;
            if(!cvWriter.open(argv[2], cvReader.get(CV_CAP_PROP_FOURCC), cvReader.get(CV_CAP_PROP_FPS), cv::Size(width, height)))
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
            vxImages.push_back(out);
        counter++;
        if(counter == 100) break;
        std::cout << counter << " processed frames" << std::endl;
    }

    vstub.EnableCuting(width, height);

    cv::Mat cvimg;
    for(int i = 0; i < vxImages.size(); i++)
    {
        vx_image img = vxImages[i];
        vx_image cuted = vstub.CutImage(img);
        if(!VX2CV(cuted, cvimg))
        {
            printf("Can't convert image VX->CV. Stop!\n");
            break;
        }
        cvWriter << cvimg;
        std::cout << i << " cuted frames" << std::endl;
    }

    cvWriter.release();
    printf("**** Performance ****\n");
    vstub.PrintPerf();
    printf("*********************\n");
    return 0;
}
