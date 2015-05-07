#include "add_kernels.h"
#include "vx_internal.h"

static vx_status VX_CALLBACK vxCutKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_status status = VX_SUCCESS;
    vx_image input = (vx_image)parameters[0];
    vx_image output = (vx_image)parameters[5];
    vx_scalar scalar[4];
    vx_uint32 pnts[4];
    vx_rectangle_t rect, dst_rect;
    vx_imagepatch_addressing_t dst_addr, src_addr;
    void *dst_buff = NULL, *src_buff = NULL;

    int i, x, y;
    for(i = 0; i < 4; i++)
    {
       scalar[i] = (vx_scalar)parameters[i + 1];
       status |= vxAccessScalarValue(scalar[i], &pnts[i]);
       status |= vxCommitScalarValue(scalar[i], &pnts[i]);
    }
    rect.start_x = pnts[0]; rect.start_y = pnts[1];
    rect.end_x = pnts[2]; rect.end_y = pnts[3];
    status |= vxGetValidRegionImage(output, &dst_rect);
    status |= vxAccessImagePatch(input, &rect, 0, &src_addr, (void **)&src_buff, VX_READ_AND_WRITE);
    status |= vxAccessImagePatch(output, &dst_rect, 0, &dst_addr, (void **)&dst_buff, VX_READ_AND_WRITE);
    for (y = 0; y < src_addr.dim_y; y++)
    {
        for (x = 0; x < src_addr.dim_x; x++)
        {
            vx_uint8* dst = (vx_uint8*)vxFormatImagePatchAddress2d(dst_buff, x, y, &dst_addr);
            vx_uint8* src = (vx_uint8*)vxFormatImagePatchAddress2d(src_buff, x, y, &src_addr);
            memcpy(dst, src, 3);
        }
    }
    status |= vxCommitImagePatch(input, NULL, 0, &src_addr, src_buff);
    status |= vxCommitImagePatch(output, &dst_rect, 0, &dst_addr, dst_buff);
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
    if (index > 0 && index < 5)
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
                if ((data_type == VX_TYPE_UINT32))
                {
                   status = VX_SUCCESS;
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
    if (index == 5)
    {
       vx_parameter params[4];
       vx_uint32 pnts[4];
       int i;
       for(i = 0; i < 4; i++)
       {
          status = VX_ERROR_INVALID_PARAMETERS;
          params[i] = vxGetParameterByIndex(node, i + 1);
          if (params[i])
          {
             vx_scalar scalar;
             vxQueryParameter(params[i], VX_PARAMETER_ATTRIBUTE_REF, &scalar, sizeof(scalar));
             if (scalar)
             {
                vxAccessScalarValue(scalar, &pnts[i]);
                status = VX_SUCCESS;
                vxCommitScalarValue(scalar, &pnts[i]);
             }
          }
          vxReleaseParameter(&params[i]);
       }
       if(status != VX_SUCCESS)
          return status;
       ptr->type = VX_TYPE_IMAGE;
       ptr->dim.image.format = VX_DF_IMAGE_RGB;
       ptr->dim.image.width = pnts[2] - pnts[0];
       ptr->dim.image.height = pnts[3] - pnts[1];
    }
    return status;
}

static vx_param_description_t add_cut_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
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

