#include "cv_tools.h"

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

bool Converter(vx_image vxImage, cv::Mat& cvImage, ConvKernel kernel)
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

bool VX2CV(vx_image vxImage, cv::Mat& cvImage)
{
    if(cvImage.empty())
    {
        vx_uint32 width = 0, height = 0;
        vxQueryImage(vxImage, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
        vxQueryImage(vxImage, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
        cvImage.create(height, width, CV_8UC3);
    }
    return Converter(vxImage, cvImage, &_VX2CV);
}

cv::Mat MergeImage(cv::Mat up, cv::Mat down)
{
    if(up.cols != down.cols || up.rows != down.rows || up.type() != down.type())
    {
        printf("Can't merge images with different types or sizes!");
        return cv::Mat();
    }
    cv::Mat res(up.rows * 2, up.cols, up.type());

    cv::Mat upRoi = res(cv::Rect(0, 0, up.cols, up.rows));
    cv::Mat downRoi = res(cv::Rect(0, up.rows, up.cols, up.rows));

    cv::Mat upImg = up(cv::Rect(0, 0, up.cols, up.rows));
    cv::Mat downImg = down(cv::Rect(0, 0, down.cols, down.rows));

    upImg.copyTo(upRoi);
    downImg.copyTo(downRoi);
    return res;
}

