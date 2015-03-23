#include "add_kernels.h"
#include "vx_internal.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

static vx_status VX_CALLBACK vxMatrixInvertKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_status status = VX_SUCCESS;
    vx_matrix input = (vx_matrix)parameters[0];
    vx_matrix output = (vx_matrix)parameters[1];

    vx_float32 in_matr[9], out_matr[9];
    status |= vxAccessMatrix(input, in_matr);
    status |= vxAccessMatrix(output, out_matr);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Cann't access to matrix(%d)!\n", status);
        return status;
    }

    /*** CV invert matrix ***/
    cv::Mat_<float> cv_matr(3, 3);
    memcpy(cv_matr.data, in_matr, sizeof(vx_float32) * 9);
    cv_matr = cv_matr.inv();
    memcpy(out_matr, cv_matr.data,sizeof(vx_float32) * 9);
    printf("\n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
        out_matr[0],out_matr[1],out_matr[2],
        out_matr[3],out_matr[4],out_matr[5],
        out_matr[6],out_matr[7],out_matr[8]);
    /************************/

    status |= vxCommitMatrix(input, in_matr);
    status |= vxCommitMatrix(output, out_matr);
    return status;
}


static vx_status VX_CALLBACK vxMatrixInvertInputValidator(vx_node node, vx_uint32 index)
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
    return status;
}

static vx_status VX_CALLBACK vxMatrixInvertOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 1)
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

static vx_param_description_t add_matrix_invert_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_matrix_invert_kernel = {
    VX_ADD_KERNEL_MATRIX_INVERT,
    VX_ADD_KERNEL_NAME_MATRIX_INVERT,
    vxMatrixInvertKernel,
    add_matrix_invert_kernel_params, dimof(add_matrix_invert_kernel_params),
    vxMatrixInvertInputValidator,
    vxMatrixInvertOutputValidator,
    NULL, NULL
};

