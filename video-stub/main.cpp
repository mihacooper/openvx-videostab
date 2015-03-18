#include <cstdio>
#include "vx_module.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"

bool CV2VX(vx_image vxImage, cv::Mat cvImage)
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
            vx_uint8* src = ptr + 3 * x;
            vx_uint8* dst = (vx_uint8*)vxFormatImagePatchAddress2d(buff, x, y, &addr);
            dst[0] = src[2]; // r
            dst[1] = src[1]; // g
            dst[2] = src[0]; // b
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

bool VX2CV(vx_image vxImage, cv::Mat cvImage)
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
            vx_uint8* dst = ptr + 3 * x;
            vx_uint8* src = (vx_uint8*)vxFormatImagePatchAddress2d(buff, x, y, &addr);
            dst[0] = src[2]; // b
            dst[1] = src[1]; // g
            dst[2] = src[0]; // r
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

#define WINDOW_NAME "VideoStub"

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("Use ./%s <video_name>\n", argv[0]);
        return 0;
    }

    vx_size gauss_size = 5;
    cv::VideoCapture cvReader(argv[1]);
    VXVideoStub vstub;
    vstub.EnableDebug({VX_ZONE_ERROR, VX_ZONE_WARNING/*, VX_ZONE_LOG, VX_ZONE_DELAY, VX_ZONE_IMAGE*/});
    bool first = true;
    cv::Mat cvIMage;
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
            if(!VX2CV(vxImage, cvIMage))
            {
                printf("Can't convert image VX->CV. Stop!\n");
                break;
            }
        }
        cv::imshow(WINDOW_NAME, cvIMage);
        cv::waitKey(5);
    }
    return 0;
}
