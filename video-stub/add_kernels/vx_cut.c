#include "add_kernels.h"
#include "vx_internal.h"

static vx_status VX_CALLBACK vxCutKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_status status = VX_SUCCESS;
    vx_image input = (vx_image)parameters[0];
    vx_image output = (vx_image)parameters[2];
    vx_scalar scalar  = (vx_scalar)parameters[1];
    vx_rectangle_t src_rect, dst_rect;
    vx_imagepatch_addressing_t dst_addr, src_addr;
    void *dst_buff = NULL, *src_buff = NULL;

    status = vxAccessScalarValue(scalar, &src_rect);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Cann't access to input parameters(%d)!\n", status);
        return status;
    }

    status  = vxGetValidRegionImage(output, &dst_rect);
    status |= vxAccessImagePatch(input, &src_rect, 0, &src_addr, (void **)&src_buff, VX_READ_AND_WRITE);
    status |= vxAccessImagePatch(output, &dst_rect, 0, &dst_addr, (void **)&dst_buff, VX_READ_AND_WRITE);
    status |= vxCommitImagePatch(input, NULL, 0, &src_addr, src_buff);
    status |= vxCommitImagePatch(output, &dst_rect, 0, &dst_addr, src_buff);
    status |= vxCommitScalarValue(scalar, &src_rect);
    return status;
}


static vx_status VX_CALLBACK vxCutInputValidator(vx_node node, vx_uint32 index)
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
    if (index == 1)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_scalar scalar;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar, sizeof(scalar));
            if (scalar)
            {
                vx_enum data_type = 0;
                vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &data_type, sizeof(data_type));
                if ((data_type == VX_TYPE_RECTANGLE))
                {
                    vx_rectangle_t rect = {0, 0, 0, 0};
                    vxAccessScalarValue(scalar, &rect);
                    if(rect.start_x >= 0 && rect.start_y >= 0
                       && rect.end_x > rect.start_x && rect.end_y >= rect.start_y)
                    {
                        status = VX_SUCCESS;
                    }
                    vxCommitScalarValue(scalar, &rect);
                }
                vxReleaseScalar(&scalar);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxCutOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 2)
    {
        vx_parameter param = vxGetParameterByIndex(node, 1);
        if (param)
        {
            vx_scalar scalar;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar, sizeof(scalar));
            if (scalar)
            {
                vx_rectangle_t rect = {0, 0, 0, 0};
                vxAccessScalarValue(scalar, &rect);
                vx_uint32 width = rect.end_x - rect.start_x,
                          height = rect.end_y - rect.start_y;
                ptr->type = VX_TYPE_IMAGE;
                ptr->dim.image.format = VX_DF_IMAGE_RGB;
                ptr->dim.image.width = width;
                ptr->dim.image.height = height;
                status = VX_SUCCESS;
                vxCommitScalarValue(scalar, &rect);
            }
            vxReleaseParameter(&param);
        }
    }
    return VX_SUCCESS;
}

static vx_param_description_t add_cut_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_cut_kernel = {
    VX_ADD_KERNEL_CUT,
    VX_ADD_KERNEL_NAME_CUT,
    vxCutKernel,
    add_cut_kernel_params, dimof(add_cut_kernel_params),
    vxCutInputValidator,
    vxCutOutputValidator,
    NULL, NULL
};

