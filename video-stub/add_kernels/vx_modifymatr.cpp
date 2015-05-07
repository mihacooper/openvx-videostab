#include "add_kernels.h"
#include "vx_internal.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#define ACCURACY 0.01

static void TransformPerspective(vx_coordinates2d_t point, vx_coordinates2d_t& tpoint, const vx_float32 m[])
{
    vx_float32 z = point.x * m[6] + point.y * m[7] + m[8];
    tpoint.x = (point.x * m[0] + point.y * m[1] + m[2]) / z;
    tpoint.y = (point.x * m[3] + point.y * m[4] + m[5]) / z;
}

static bool CheckPointArea(vx_coordinates2d_t point, vx_rectangle_t rect)
{
    if(point.x < rect.start_x || point.x >= rect.end_x ||
            point.y < rect.start_y || point.y >= rect.end_y)
        return false;
    return true;
}

static bool CheckRectArea(vx_coordinates2d_t points[4], vx_rectangle_t rect, const vx_float32 m[])
{
    bool in_area = true;
    for(int i = 0; i < 4; i++)
    {
        TransformPerspective(points[i], points[i], m);
        in_area &= CheckPointArea(points[i], rect);
    }
    return in_area;
}

static vx_status ModifyMatrix(vx_matrix in_matr, vx_matrix out_matr, vx_uint32 width, vx_uint32 height, vx_float32 scale)
{
    vx_rectangle_t rect = { 0, 0, width, height};
    vx_uint32 w_mod = (width * (1 - scale)) / 2.;
    vx_uint32 h_mod = (height * (1 - scale)) / 2.;
    vx_coordinates2d_t srect[4] = { {w_mod, h_mod}, {width - w_mod, h_mod},
                                    {width - w_mod, height - h_mod}, {w_mod, height - h_mod}};
    vx_float32 m[9];
    vx_float32 res[9];
    vx_float32 inv[9];
    vxAccessMatrix(in_matr, m);
    vxAccessMatrix(out_matr, res);

    // invert
    cv::Mat_<float> cv_matr(3, 3), cv_inv(3, 3);
    memcpy(cv_matr.data, m, sizeof(vx_float32) * 9);
    cv_inv = cv_matr.inv();
    memcpy(inv, cv_inv.data,sizeof(vx_float32) * 9);
    // check need modify
    bool do_modify = !CheckRectArea(srect, rect, inv);

    if(do_modify)
    {
        vx_float32 left = 0., right = 1.;
        cv::Mat_<float> cv_eye = cv::Mat::eye(3, 3, CV_32FC1);
        cv::Mat_<float> cv_res(3, 3);
        while(right - left > ACCURACY)
        {
            vx_float32 alpha = (left + right) / 2.;
            cv_res = alpha * cv_matr + (1. - alpha) * cv_eye;
            cv_inv = cv_res.inv();
            bool check = CheckRectArea(srect, rect, (vx_float32*)cv_inv.data);
            if(check) left  = alpha;
            else      right = alpha;
        }
        memcpy(res, cv_res.data, sizeof(vx_float32) * 9);
    }
    else
    {
        memcpy(res, m, sizeof(vx_float32) * 9);
    }
    vxCommitMatrix(in_matr, m);
    vxCommitMatrix(out_matr, res);
    return VX_SUCCESS;
}

static vx_status VX_CALLBACK vxMatrixModifyKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_status status = VX_SUCCESS;
    vx_matrix in_matr  = (vx_matrix)parameters[0];
    vx_scalar s_width  = (vx_scalar)parameters[1];
    vx_scalar s_height = (vx_scalar)parameters[2];
    vx_scalar s_scale  = (vx_scalar)parameters[3];
    vx_matrix out_matr = (vx_matrix)parameters[4];

    vx_uint32 width, height;
    vx_float32 scale;
    status |= vxAccessScalarValue(s_width,  &width);
    status |= vxAccessScalarValue(s_height, &height);
    status |= vxAccessScalarValue(s_scale,  &scale);
    if(status == VX_SUCCESS)
    {
        status = ModifyMatrix(in_matr, out_matr, width, height, scale);
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Access scalar values failure!\n");
    }
    return status;
}

static vx_status VX_CALLBACK vxMatrixModifyInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0)
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
    if (index == 1 || index == 2)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_scalar scalar = 0;
            status = vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar, sizeof(scalar));
            if ((status == VX_SUCCESS) && (scalar))
            {
                vx_enum type = VX_TYPE_INVALID;
                vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
                if (type == VX_TYPE_UINT32)
                {
                    status = VX_SUCCESS;
                }
                else
                {
                    status = VX_ERROR_INVALID_TYPE;
                }
                vxReleaseScalar(&scalar);
            }
            vxReleaseParameter(&param);
        }
    }
    if (index == 3)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_scalar scalar = 0;
            status = vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar, sizeof(scalar));
            if ((status == VX_SUCCESS) && (scalar))
            {
                vx_enum type = VX_TYPE_INVALID;
                vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
                if (type == VX_TYPE_FLOAT32)
                {
                    status = VX_SUCCESS;
                }
                else
                {
                    status = VX_ERROR_INVALID_TYPE;
                }
                vxReleaseScalar(&scalar);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxMatrixModifyOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t* ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 4)
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
    return VX_SUCCESS;
}

static vx_param_description_t add_modify_matrix_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_modify_matrix_kernel = {
    VX_ADD_KERNEL_MATRIX_MODIFY,
    VX_ADD_KERNEL_NAME_MATRIX_MODIFY,
    vxMatrixModifyKernel,
    add_modify_matrix_kernel_params, dimof(add_modify_matrix_kernel_params),
    vxMatrixModifyInputValidator,
    vxMatrixModifyOutputValidator,
    NULL, NULL
};

