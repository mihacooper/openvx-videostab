#include "add_kernels.h"
#include "vx_internal.h"
#include "opencv2/calib3d/calib3d.hpp"
#include <vector>

static vx_status VX_CALLBACK vxFindWarpKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    if(num != 3)
        return VX_ERROR_INVALID_PARAMETERS;

    vx_status status = VX_SUCCESS;
    vx_array def_pnts   = (vx_array) parameters[0];
    vx_array moved_pnts = (vx_array) parameters[1];
    vx_matrix matrix    = (vx_matrix)parameters[2];

    vx_size points_num;
    status |= vxQueryArray(def_pnts, VX_ARRAY_ATTRIBUTE_NUMITEMS, &points_num, sizeof(points_num));
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't query array attribute(%d)!\n", status);
        return VX_FAILURE;
    }
    
    if(points_num < 4)
    {
        vx_float32 matr_buff[9];
        vxAccessMatrix(matrix, (void*)matr_buff);
        memset(matr_buff, 0, sizeof(vx_float32) * 9);
        matr_buff[0] = matr_buff[4] = matr_buff[8] = 1.;
        vxCommitMatrix(matrix, (void*)matr_buff);
        VX_PRINT(VX_ZONE_WARNING, "Number of points less then 4(%d)!\n", points_num);
        return VX_SUCCESS;//VX_FAILURE;
    }

    /*** CV array initialize ***/
    std::vector<cv::Point2f> cv_points_from, cv_points_to;
    cv_points_from.reserve(points_num);
    cv_points_to.reserve(points_num);
    /***************************/

    vx_size i, stride1 = 0ul, stride2 = 0ul;
    void *def_buff = NULL, *moved_buff = NULL;
    status |= vxAccessArrayRange(def_pnts, 0, points_num, &stride1, &def_buff, VX_READ_AND_WRITE);
    status |= vxAccessArrayRange(moved_pnts, 0, points_num, &stride2, &moved_buff, VX_READ_AND_WRITE);

    if(def_buff && moved_buff)
    {
        for (i = 0; i < points_num; i++)
        {
            if(vxArrayItem(vx_keypoint_t, moved_buff, i, stride2).tracking_status)
            {
                cv::Point2f pnt_from;
                pnt_from.x = vxArrayItem(vx_keypoint_t, def_buff, i, stride1).x;
                pnt_from.y = vxArrayItem(vx_keypoint_t, def_buff, i, stride1).y;
                cv_points_from.push_back(pnt_from);

                cv::Point2f pnt_to;
                pnt_to.x = vxArrayItem(vx_keypoint_t, moved_buff, i, stride2).x;
                pnt_to.y = vxArrayItem(vx_keypoint_t, moved_buff, i, stride2).y;
                cv_points_to.push_back(pnt_to);
            }
        }
    }
    else
    {
        status = VX_FAILURE;
    }
    status |= vxCommitArrayRange(def_pnts, 0, points_num, def_buff);
    status |= vxCommitArrayRange(moved_pnts, 0, points_num, moved_buff);
    VX_PRINT(VX_ZONE_LOG, "Number of points = (%d)!\n", cv_points_from.size());

    /*** CV find homography ***/
    cv::Mat_<float> cv_matr = cv::Mat::eye(3, 3, CV_32FC1);
    if(cv_points_from.size() > 0 && cv_points_to.size() > 0)
        cv_matr = cv::findHomography(cv_points_from, cv_points_to, CV_RANSAC);
    else
        VX_PRINT(VX_ZONE_WARNING, "Number of points is equal to zero!\n");
    vx_float32 matr_buff[9];
    status |= vxAccessMatrix(matrix, (void*)matr_buff);
    memcpy(matr_buff, cv_matr.data, sizeof(vx_float32) * 9);

    status |= vxCommitMatrix(matrix, (void*)matr_buff);
    return status;
}

static vx_status VX_CALLBACK vxFindWarpInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0 || index == 1 )
    {
        vx_parameter param1 = vxGetParameterByIndex(node, 0);
        vx_parameter param2 = vxGetParameterByIndex(node, 1);
        if (param1 && param2)
        {
            vx_array arr1 = 0, arr2 = 0;
            vxQueryParameter(param1, VX_PARAMETER_ATTRIBUTE_REF, &arr1, sizeof(arr1));
            vxQueryParameter(param2, VX_PARAMETER_ATTRIBUTE_REF, &arr2, sizeof(arr2));
            if (arr1 && arr2)
            {
                vx_enum item_type1 = 0, item_type2 = 0;
                vxQueryArray(arr1, VX_ARRAY_ATTRIBUTE_ITEMTYPE, &item_type1, sizeof(item_type1));
                vxQueryArray(arr2, VX_ARRAY_ATTRIBUTE_ITEMTYPE, &item_type2, sizeof(item_type2));
                if (item_type1 == VX_TYPE_KEYPOINT && item_type2 == VX_TYPE_KEYPOINT)
                {
                    vx_size num1, num2;
                    vxQueryArray(arr1, VX_ARRAY_ATTRIBUTE_NUMITEMS, &num1, sizeof(num1));
                    vxQueryArray(arr2, VX_ARRAY_ATTRIBUTE_NUMITEMS, &num2, sizeof(num2));
                    if(num1 == num2)
                    {
                        status = VX_SUCCESS;
                    }
                }
                vxReleaseArray(&arr1);
                vxReleaseArray(&arr2);
            }
            vxReleaseParameter(&param1);
            vxReleaseParameter(&param2);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxFindWarpOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 2)
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

static vx_param_description_t add_find_warp_kernel_params[] = {
    {VX_INPUT, VX_TYPE_ARRAY, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT, VX_TYPE_ARRAY, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_find_warp_kernel = {
    VX_ADD_KERNEL_FIND_WARP,
    VX_ADD_KERNEL_NAME_FIND_WARP,
    vxFindWarpKernel,
    add_find_warp_kernel_params, dimof(add_find_warp_kernel_params),
    vxFindWarpInputValidator, vxFindWarpOutputValidator,
    NULL, NULL
};
