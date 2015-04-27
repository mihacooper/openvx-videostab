#include "add_kernels.h"
#include "vx_internal.h"
#include "opencv2/video/video.hpp"

static vx_status VX_CALLBACK vxCVOptFlowKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_status status = VX_SUCCESS;
    vx_image old_image = (vx_image)parameters[0];
    vx_image new_image = (vx_image)parameters[1];
    vx_array input_arr = (vx_array)parameters[2];
    vx_array output_arr = (vx_array)parameters[3];

    vx_rectangle_t old_rect, new_rect;
    vx_imagepatch_addressing_t old_addr, new_addr;
    void *old_buff = NULL, *new_buff = NULL;

    status |= vxGetValidRegionImage(old_image, &old_rect);
    status |= vxGetValidRegionImage(new_image, &new_rect);
    status |= vxAccessImagePatch(old_image, &old_rect, 0, &old_addr, (void **)&old_buff, VX_READ_AND_WRITE);
    status |= vxAccessImagePatch(new_image, &new_rect, 0, &new_addr, (void **)&new_buff, VX_READ_AND_WRITE);
    cv::Mat cv_old_image(cv::Size(old_addr.dim_x, old_addr.dim_y), CV_8UC1, old_buff);
    cv::Mat cv_new_image(cv::Size(new_addr.dim_x, new_addr.dim_y), CV_8UC1, new_buff);

    vx_size elements; vx_size stride = 0;
    vxQueryArray(input_arr, VX_ARRAY_ATTRIBUTE_NUMITEMS, &elements, sizeof(elements));
    vx_keypoint_t* in_arr_buff = (vx_keypoint_t*)malloc(elements * sizeof(vx_keypoint_t));
    vx_keypoint_t* out_arr_buff = (vx_keypoint_t*)malloc(elements * sizeof(vx_keypoint_t));
    vxAccessArrayRange(input_arr, 0, elements, &stride, (void**)&in_arr_buff, VX_READ_AND_WRITE);
    std::vector<cv::Point2f> in_cv_arr(elements);
    for(int i = 0; i < elements; i++)
    {
        in_cv_arr[i].x = vxArrayItem(vx_keypoint_t, in_arr_buff, i, stride).x;
        in_cv_arr[i].y = vxArrayItem(vx_keypoint_t, in_arr_buff, i, stride).y;
    }
    std::vector<cv::Point2f> out_cv_arr;
    std::vector<uchar> v_status;
    cv::calcOpticalFlowPyrLK(cv_old_image, cv_new_image, in_cv_arr, out_cv_arr, v_status, cv::noArray());
    for(int i = 0; i < elements; i++)
    {
        vxArrayItem(vx_keypoint_t, out_arr_buff, i, stride).x = out_cv_arr[i].x;
        vxArrayItem(vx_keypoint_t, out_arr_buff, i, stride).y = out_cv_arr[i].y;
        vxArrayItem(vx_keypoint_t, out_arr_buff, i, stride).tracking_status = (v_status[i] ? 1 : 0);
    }
    status |= vxTruncateArray(output_arr, 0);
    status |= vxAddArrayItems(output_arr, elements, out_arr_buff, stride);
    status |= vxCommitArrayRange(input_arr, 0, elements, in_arr_buff);
    free(in_arr_buff); free(out_arr_buff);

    status |= vxCommitImagePatch(old_image, NULL, 0, &old_addr, old_buff);
    status |= vxCommitImagePatch(new_image, NULL, 0, &new_addr, new_buff);
    return status;
}


static vx_status VX_CALLBACK vxCVOptFlowInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0 || index == 1)
    {
        vx_image input = 0;
        vx_parameter param = vxGetParameterByIndex(node, index);
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(input));
        if (input)
        {
            vx_df_image format = 0;
            vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
            if (format == VX_DF_IMAGE_U8)
                status = VX_SUCCESS;
            vxReleaseImage(&input);
        }
        vxReleaseParameter(&param);
    }
    else if (index == 2)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_array arr = 0;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &arr, sizeof(arr));
            if (arr)
            {
                vx_enum item_type = 0;
                vxQueryArray(arr, VX_ARRAY_ATTRIBUTE_ITEMTYPE, &item_type, sizeof(item_type));
                if (item_type == VX_TYPE_KEYPOINT)
                {
                    status = VX_SUCCESS;
                }
                vxReleaseArray(&arr);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxCVOptFlowOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 3)
    {
        vx_array arr = 0;
        vx_size capacity = 0;
        vx_parameter param = vxGetParameterByIndex(node, 2);
        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &arr, sizeof(arr));
        vxQueryArray(arr, VX_ARRAY_ATTRIBUTE_CAPACITY, &capacity, sizeof(capacity));

        ptr->type = VX_TYPE_ARRAY;
        ptr->dim.array.item_type = VX_TYPE_KEYPOINT;
        ptr->dim.array.capacity = capacity;

        status = VX_SUCCESS;

        vxReleaseArray(&arr);
        vxReleaseParameter(&param);
    }
    return status;
}

static vx_param_description_t add_cv_optflow_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_IMAGE,  VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_ARRAY,  VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_ARRAY,  VX_PARAMETER_STATE_REQUIRED}
};

vx_kernel_description_t add_cv_optflow_kernel = {
    VX_ADD_KERNEL_CV_OPTFLOW,
    VX_ADD_KERNEL_NAME_CV_OPTFLOW,
    vxCVOptFlowKernel,
    add_cv_optflow_kernel_params, dimof(add_cv_optflow_kernel_params),
    vxCVOptFlowInputValidator,
    vxCVOptFlowOutputValidator,
    NULL, NULL
};

