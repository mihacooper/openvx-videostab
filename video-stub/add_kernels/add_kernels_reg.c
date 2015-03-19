#include "add_kernels.h"

extern vx_kernel_description_t add_rgb_to_gray_kernel;
extern vx_kernel_description_t add_find_warp_kernel;
extern vx_kernel_description_t add_warp_perspective_rgb_kernel;
extern vx_kernel_description_t add_matrix_multiply_kernel;

static vx_kernel_description_t* add_kernels[] = {
    &add_rgb_to_gray_kernel,
    &add_find_warp_kernel,
    &add_warp_perspective_rgb_kernel,
    &add_matrix_multiply_kernel
};

static vx_uint32 num_add_kernels = dimof(add_kernels);

VX_SAMPLE_API vx_status vxPublishKernels(vx_context context)
{
    vx_status status = VX_SUCCESS;
    int i = 0, p = 0;
    vx_kernel_description_t* kernelDesc = NULL;
    vx_kernel kernel = NULL;

    for (i = 0; i < num_add_kernels && status == VX_SUCCESS; i++)
    {
        kernelDesc = add_kernels[i];
        kernel = vxAddKernel(context,
                             kernelDesc->name,
                             kernelDesc->enumeration,
                             kernelDesc->function,
                             kernelDesc->numParams,
                             kernelDesc->input_validate,
                             kernelDesc->output_validate,
                             kernelDesc->initialize,
                             kernelDesc->deinitialize);
        if (kernel)
        {
            for (p = 0; p < kernelDesc->numParams; p++)
            {
                status = vxAddParameterToKernel(kernel, p, kernelDesc->parameters[p].direction,
                                                kernelDesc->parameters[p].data_type, kernelDesc->parameters[p].state);
                if (status != VX_SUCCESS)
                {
                    vxRemoveKernel(kernel);
                }
            }

            if (status == VX_SUCCESS)
            {
                status = vxFinalizeKernel(kernel);
                if (status != VX_SUCCESS)
                    if (status != VX_SUCCESS)
                    {
                        vxRemoveKernel(kernel);
                    }
            }
        }
        else
        {
            status = VX_ERROR_INVALID_PARAMETERS;
        }
    }
    return status;
}


