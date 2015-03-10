#include "vreader.h"
#include <cstdio>

#include "opencv2/imgproc/imgproc.hpp"

VReader::VReader(std::string filename) : m_cvReader(filename.c_str()) {}

VReader::~VReader() {}

bool VReader::ReadFrame(vx_context context, vx_image& frame)
{
    cv::Mat image;
    m_cvReader >> image;
    if(image.empty())
        return false;

    if(image.type() != CV_8UC3)
    {
        printf("Unsupported image type: %d", image.type());
    }

    vx_status status = VX_SUCCESS;
    frame = vxCreateImage(context, image.cols, image.rows, VX_DF_IMAGE_RGB);

    vx_uint32 y, x, width = 0, height = 0;
    void *buff = NULL;
    vx_imagepatch_addressing_t addr;
    vx_rectangle_t rect = {0, 0, image.cols, image.rows};
    status |= vxAccessImagePatch(frame, &rect, 0, &addr, (void **)&buff, VX_READ_AND_WRITE);
    height = addr.dim_y;
    width = addr.dim_x;

    for (y = 0; y < height; y++)
    {
        uchar* ptr = (uchar*) (image.data + y * image.step[0]);
        for (x = 0; x < width; x++)
        {
            vx_int32 cv_val[3] = { ptr[3 * x], ptr[3 * x + 1], ptr[3 * x + 2]};
            vx_uint32* pix = (vx_uint32*)vxFormatImagePatchAddress2d(buff, x, y, &addr);
            *pix = ( (cv_val[2] << 16) +  (cv_val[1] << 8) + cv_val[0]) << 8;
        }
    }
    status |= vxCommitImagePatch(frame, NULL, 0, &addr, buff);
    return true;
}

void VReader::ShowFrame(vx_image& frame)
{
    /*
    cv::Mat image();
    vx_status status = VX_SUCCESS;
    frame = vxCreateImage(context, image.cols, image.rows, VX_DF_IMAGE_RGB);

    vx_uint32 y, x, width = 0, height = 0;
    void *buff = NULL;
    vx_imagepatch_addressing_t addr;
    vx_rectangle_t rect = {0, 0, image.cols, image.rows};
    status |= vxAccessImagePatch(frame, &rect, 0, &addr, (void **)&buff, VX_READ_AND_WRITE);
    height = addr.dim_y;
    width = addr.dim_x;

    for (y = 0; y < height; y++)
    {
        uchar* ptr = (uchar*) (image.data + y * image.step[0]);
        for (x = 0; x < width; x++)
        {
            vx_int32 cv_val[3] = { ptr[3 * x], ptr[3 * x + 1], ptr[3 * x + 2]};
            vx_uint32* pix = (vx_uint32*)vxFormatImagePatchAddress2d(buff, x, y, &addr);
            *pix = ( (cv_val[2] << 16) +  (cv_val[1] << 8) + cv_val[0]) << 8;
        }
    }
    status |= vxCommitImagePatch(frame, NULL, 0, &addr, buff);
    */
}

