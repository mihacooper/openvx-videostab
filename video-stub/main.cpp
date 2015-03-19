#include <cstdio>
#include <iostream>
#include "vx_module.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

void _CV2VX(vx_uint8* cv, vx_uint8* vx)
{
    vx[0] = cv[2]; // r
    vx[1] = cv[1]; // g
    vx[2] = cv[0]; // b
}

void _VX2CV(vx_uint8* cv, vx_uint8* vx)
{
    cv[0] = vx[2]; // b
    cv[1] = vx[1]; // g
    cv[2] = vx[0]; // r
}

typedef void(*ConvKernel)(vx_uint8* a, vx_uint8* b);

bool Converter(vx_image vxImage, cv::Mat cvImage, ConvKernel kernel)
{
    if(cvImage.type() != CV_8UC3)
    {
        printf("Unsupported image type: %d", cvImage.type());
        return false;
    }

    vx_status status = VX_SUCCESS;

    vx_uint32 width = 0, height = 0;
    vx_df_image format;
    status |= vxQueryImage(vxImage, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
    status |= vxQueryImage(vxImage, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
    status |= vxQueryImage(vxImage, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
    if(status != VX_SUCCESS)
    {
        printf("Can't query image attribute(%d)!\n", status);
        return false;
    }
    if(cvImage.cols != width || cvImage.rows != height || format != VX_DF_IMAGE_RGB )
    {
        printf("VX and CV image formats aren't equal!");
        return false;
    }

    void *buff = NULL;
    vx_rectangle_t rect = {0, 0, width, height};
    vx_imagepatch_addressing_t addr;
    status |= vxAccessImagePatch(vxImage, &rect, 0, &addr, &buff, VX_READ_AND_WRITE);
    if(status != VX_SUCCESS)
    {
        printf("Can't access to image patch(%d)!\n", status);
        return false;
    }

    for (int y = 0; y < height; y++)
    {
        uchar* ptr = (uchar*) (cvImage.data + y * cvImage.step[0]);
        for (int x = 0; x < width; x++)
        {
            vx_uint8* cv = ptr + 3 * x;
            vx_uint8* vx = (vx_uint8*)vxFormatImagePatchAddress2d(buff, x, y, &addr);
            (*kernel)(cv, vx);
        }
    }

    status |= vxCommitImagePatch(vxImage, &rect, 0, &addr, buff);
    if(status != VX_SUCCESS)
    {
        printf("Can't commit image patch(%d)!\n", status);
        return false;
    }
    return true;
}

bool CV2VX(vx_image vxImage, cv::Mat cvImage)
{
    return Converter(vxImage, cvImage, &_CV2VX);
}

bool VX2CV(vx_image vxImage, cv::Mat cvImage)
{
    return Converter(vxImage, cvImage, &_VX2CV);
}

#define WINDOW_NAME "VideoStub"

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("Use ./%s <input_video> <output_video>\n", argv[0]);
        return 0;
    }

    cv::VideoCapture cvReader(argv[1]);
    cv::VideoWriter  cvWriter;

    VXVideoStub vstub;
    vx_size gauss_size = 5;
    vstub.EnableDebug({VX_ZONE_ERROR/*, VX_ZONE_LOG, VX_ZONE_DELAY, VX_ZONE_IMAGE*/});
    bool first = true;
    cv::Mat cvIMage;
    int counter = 0;
    while(true)
    {
        cvReader >> cvIMage;
        if(cvIMage.empty())
        {
            printf("End of video!\n");
            break;
        }
        if(first)
        {
            if(vstub.CreatePipeline(cvIMage.cols, cvIMage.rows, gauss_size) != VX_SUCCESS)
                break;
            //if(!cvWriter.open(argv[2], -1, cvReader.get(CV_CAP_PROP_FPS), cv::Size(cvIMage.cols, cvIMage.rows)))
            //{
            //    std::cout << " Cann't open output video file!" << std::endl;
            //    break;
            //}
            first = false;
        }
        vx_image vxImage = vstub.NewImage();
        if(!CV2VX(vxImage, cvIMage))
        {
            printf("Can't convert image CV->VX. Stop!\n");
            break;
        }
        vx_image out = vstub.Calculate();
        if(out)
        {
            if(!VX2CV(out, cvIMage))
            {
                printf("Can't convert image VX->CV. Stop!\n");
                break;
            }
        }
        cv::imshow(WINDOW_NAME, cvIMage);
        cv::waitKey(5);
        //cvWriter << cvIMage;
        counter++;
        std::cout << counter << " processed frames" << std::endl;
    }
    cvWriter.release();
    return 0;
}
