#include "add_kernels.h"
#include "vx_internal.h"

static vx_status VX_CALLBACK vxRGBtoGrayKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    if(num != 2)
        return VX_ERROR_INVALID_PARAMETERS;

    vx_image input = (vx_image)parameters[0];
    vx_image output = (vx_image)parameters[1];

    vx_uint32 y, x, width = 0, height = 0;
    void *dst_buff   = NULL, *src_buff = NULL;
    vx_status status = VX_SUCCESS;
    vx_imagepatch_addressing_t dst_addr, src_addr;
    vx_rectangle_t rect;

    status  = vxGetValidRegionImage(input, &rect);
    status |= vxAccessImagePatch(input, &rect, 0, &src_addr, (void **)&src_buff, VX_READ_AND_WRITE);
    status |= vxAccessImagePatch(output, &rect, 0, &dst_addr, (void **)&dst_buff, VX_READ_AND_WRITE);
    height = src_addr.dim_y;
    width = src_addr.dim_x;
    for (y = 0; y < height; y++)
    {
        for (x = 0; x < width; x++)
        {
            vx_uint8* src = vxFormatImagePatchAddress2d(src_buff, x, y, &src_addr);
            vx_uint8* dst = vxFormatImagePatchAddress2d(dst_buff, x, y, &dst_addr);
            *dst = src[0] * 0.299 + src[1] * 0.587 + src[2] * 0.114;
        }
    }
    status |= vxCommitImagePatch(input, NULL, 0, &src_addr, src_buff);
    status |= vxCommitImagePatch(output, NULL, 0, &dst_addr, dst_buff);
    return status;
}

static vx_status VX_CALLBACK vxRGBtoGrayInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0 )
    {
        vx_image input = 0;
        vx_parameter param = vxGetParameterByIndex(node, index);

        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(input));
        if (input)
        {
            vx_df_image format = 0;
            vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
            if (format == VX_DF_IMAGE_RGB)
                status = VX_SUCCESS;
            vxReleaseImage(&input);
        }
        vxReleaseParameter(&param);
    }
    return status;
}

static vx_status VX_CALLBACK vxRGBtoGrayOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 1)
    {
        vx_parameter param = vxGetParameterByIndex(node, 0);
        if (param)
        {
            vx_image input = 0;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(input));
            if (input)
            {
                vx_uint32 width = 0, height = 0;
                vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
                vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
                ptr->type = VX_TYPE_IMAGE;
                ptr->dim.image.format = VX_DF_IMAGE_U8;
                ptr->dim.image.width = width;
                ptr->dim.image.height = height;
                status = VX_SUCCESS;
                vxReleaseImage(&input);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_param_description_t add_rgb_to_gray_kernel_params[] = {
    {VX_INPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_rgb_to_gray_kernel = {
    VX_ADD_KERNEL_RGB_TO_GRAY,
    VX_ADD_KERNEL_NAME_RGB_TO_GRAY,
    vxRGBtoGrayKernel,
    add_rgb_to_gray_kernel_params, dimof(add_rgb_to_gray_kernel_params),
    vxRGBtoGrayInputValidator, vxRGBtoGrayOutputValidator,
    NULL, NULL
};
