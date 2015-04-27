#include "add_kernels.h"
#include "vx_internal.h"

#include "../cv_tools.h"
#include "opencv2/imgproc/imgproc.hpp"

static vx_status VX_CALLBACK vxCVWarpKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_image  src_image = (vx_image) parameters[0];
    vx_matrix matrix    = (vx_matrix)parameters[1];
    vx_image  dst_image = (vx_image) parameters[2];

    vx_status status = VX_SUCCESS;
    void *src_base = NULL;
    void *dst_base = NULL;
    vx_imagepatch_addressing_t src_addr, dst_addr;
    vx_uint32 dst_width, dst_height;
    vx_rectangle_t src_rect;
    vx_rectangle_t dst_rect;

    vx_float32 m[9];

    vxQueryImage(dst_image, VX_IMAGE_ATTRIBUTE_WIDTH, &dst_width, sizeof(dst_width));
    vxQueryImage(dst_image, VX_IMAGE_ATTRIBUTE_HEIGHT, &dst_height, sizeof(dst_height));

    vxGetValidRegionImage(src_image, &src_rect);
    dst_rect.start_x = 0;
    dst_rect.start_y = 0;
    dst_rect.end_x = dst_width;
    dst_rect.end_y = dst_height;

    //status |= vxAccessImagePatch(src_image, &src_rect, 0, &src_addr, &src_base, VX_READ_ONLY);
    //status |= vxAccessImagePatch(dst_image, &dst_rect, 0, &dst_addr, &dst_base, VX_WRITE_ONLY);
    status |= vxAccessMatrix(matrix, m);

    cv::Mat cvSrc, cvDst;
    cv::Mat_<float> cvMatr(3,3);
    memcpy(cvMatr.data, m, sizeof(vx_float32) * 9);
    VX2CV(src_image, cvSrc);

    cv::warpPerspective(cvSrc, cvDst,cvMatr, cv::Size(dst_width, dst_height));

    CV2VX(dst_image, cvDst);
    status |= vxCommitMatrix(matrix, m);
    //status |= vxCommitImagePatch(src_image, NULL, 0, &src_addr, src_base);
    //status |= vxCommitImagePatch(dst_image, &dst_rect, 0, &dst_addr, dst_base);

    return status;
}


static vx_status VX_CALLBACK vxCVWarpInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0)
    {
        vx_image input = 0;
        vx_parameter param = vxGetParameterByIndex(node, index);

        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(input));
        if (input)
        {
            vx_df_image format = 0;
            vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
            if (format == VX_DF_IMAGE_RGB)
            {
                status = VX_SUCCESS;
            }
            vxReleaseImage(&input);
        }
        vxReleaseParameter(&param);
    }
    else if (index == 1)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_matrix matrix;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &matrix, sizeof(matrix));
            if (matrix)
            {
                vx_enum data_type = 0;
                vx_size rows = 0ul, columns = 0ul;
                vxQueryMatrix(matrix, VX_MATRIX_ATTRIBUTE_TYPE, &data_type, sizeof(data_type));
                vxQueryMatrix(matrix, VX_MATRIX_ATTRIBUTE_ROWS, &rows, sizeof(rows));
                vxQueryMatrix(matrix, VX_MATRIX_ATTRIBUTE_COLUMNS, &columns, sizeof(columns));
                if ((data_type == VX_TYPE_FLOAT32) && (columns == 3) && (rows == 3))
                {
                    status = VX_SUCCESS;
                }
                vxReleaseMatrix(&matrix);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxCVWarpOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 2)
    {
        vx_parameter dst_param = vxGetParameterByIndex(node, index);
        if (dst_param)
        {
            vx_image dst = 0;
            vxQueryParameter(dst_param, VX_PARAMETER_ATTRIBUTE_REF, &dst, sizeof(dst));
            if (dst)
            {
                vx_uint32 w1 = 0, h1 = 0;
                vx_df_image f1 = VX_DF_IMAGE_VIRT;

                vxQueryImage(dst, VX_IMAGE_ATTRIBUTE_WIDTH, &w1, sizeof(w1));
                vxQueryImage(dst, VX_IMAGE_ATTRIBUTE_HEIGHT, &h1, sizeof(h1));
                vxQueryImage(dst, VX_IMAGE_ATTRIBUTE_FORMAT, &f1, sizeof(f1));
                /* output can not be virtual */
                if ((w1 != 0) && (h1 != 0) && (f1 == VX_DF_IMAGE_RGB))
                {
                    /* fill in the meta data with the attributes so that the checker will pass */
                    ptr->type = VX_TYPE_IMAGE;
                    ptr->dim.image.format = VX_DF_IMAGE_RGB;
                    ptr->dim.image.width = w1;
                    ptr->dim.image.height = h1;
                    status = VX_SUCCESS;
                }
                vxReleaseImage(&dst);
            }
            vxReleaseParameter(&dst_param);
        }
    }
    return status;
}

static vx_param_description_t add_cv_warp_kernel_params[] = {
    {VX_INPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_cv_warp_kernel = {
    VX_ADD_KERNEL_CV_WARP,
    VX_ADD_KERNEL_NAME_CV_WARP,
    vxCVWarpKernel,
    add_cv_warp_kernel_params, dimof(add_cv_warp_kernel_params),
    vxCVWarpInputValidator,
    vxCVWarpOutputValidator,
    NULL, NULL
};

