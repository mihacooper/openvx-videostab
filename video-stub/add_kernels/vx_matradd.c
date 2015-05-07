#include "add_kernels.h"
#include "vx_internal.h"

static vx_status VX_CALLBACK vxMatrixAddKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_status status = VX_SUCCESS;
    vx_matrix matrix1 = (vx_matrix)parameters[0];
    vx_matrix matrix2 = (vx_matrix)parameters[1];
    vx_matrix out_matr = (vx_matrix)parameters[3];
    vx_scalar scalar  = (vx_scalar)parameters[2];

    vx_bool use_coef = (scalar != NULL);
    vx_float32 coeff = 0.;
    if(use_coef)
        vxAccessScalarValue(scalar, &coeff);

    vx_float32 matr1[9], matr2[9], res[9];
    status |= vxAccessMatrix(matrix1, matr1);
    status |= vxAccessMatrix(matrix2, matr2);
    status |= vxAccessMatrix(out_matr, res);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Cann't access to matrix(%d)!\n", status);
        return status;
    }
    int i, j;
    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            if(use_coef)
               res[i * 3 + j] = matr1[i * 3 + j] * coeff + matr2[i * 3 + j];
            else
               res[i * 3 + j] = matr1[i * 3 + j] + matr2[i * 3 + j];
        }
    }
    if(use_coef)
        status |= vxCommitScalarValue(scalar, &coeff);
    status |= vxCommitMatrix(matrix1, matr1);
    status |= vxCommitMatrix(matrix2, matr2);
    status |= vxCommitMatrix(out_matr, res);
    return status;
}


static vx_status VX_CALLBACK vxMatrixAddInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0 || index == 1)
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
    if (index == 2)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_scalar sens = 0;
            status = vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &sens, sizeof(sens));
            if ((status == VX_SUCCESS) && (sens))
            {
                vx_enum type = VX_TYPE_INVALID;
                vxQueryScalar(sens, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));
                if (type == VX_TYPE_FLOAT32)
                {
                    status = VX_SUCCESS;
                }
                else
                {
                    status = VX_ERROR_INVALID_TYPE;
                }
                vxReleaseScalar(&sens);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxMatrixAddOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 3)
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

static vx_param_description_t add_matrix_add_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_OPTIONAL},
    {VX_OUTPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_matrix_add_kernel = {
    VX_ADD_KERNEL_MATRIX_ADD,
    VX_ADD_KERNEL_NAME_MATRIX_ADD,
    vxMatrixAddKernel,
    add_matrix_add_kernel_params, dimof(add_matrix_add_kernel_params),
    vxMatrixAddInputValidator,
    vxMatrixAddOutputValidator,
    NULL, NULL
};

