#include "add_kernels.h"
#include "vx_internal.h"

static vx_status VX_CALLBACK vxMatrixMultiplyKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
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

    printf("Mul1:\n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
        matr1[0],matr1[1],matr1[2],
        matr1[3],matr1[4],matr1[5],
        matr1[6],matr1[7],matr1[8]);
    printf("Mul2:\n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
        matr2[0],matr2[1],matr2[2],
        matr2[3],matr2[4],matr2[5],
        matr2[6],matr2[7],matr2[8]);
    int i, j, k;
    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 3; j++)
        {
            res[i * 3 + j] = 0.;
            for(k = 0; k < 3; k++)
            {
                res[i * 3 + j] += matr1[i * 3 + k] * matr2[k * 3 + j];
            }
            if(use_coef)
                res[i * 3 + j] *= coeff;
        }
    }
    printf("Mul:\n%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
        res[0],res[1],res[2],
        res[3],res[4],res[5],
        res[6],res[7],res[8]);

    if(use_coef)
        status |= vxCommitScalarValue(scalar, &coeff);
    status |= vxCommitMatrix(matrix1, matr1);
    status |= vxCommitMatrix(matrix2, matr2);
    status |= vxCommitMatrix(out_matr, res);
    return status;
}


static vx_status VX_CALLBACK vxMatrixMultiplyInputValidator(vx_node node, vx_uint32 index)
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

static vx_status VX_CALLBACK vxMatrixMultiplyOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
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

static vx_param_description_t add_matrix_multiply_kernel_params[] = {
    {VX_INPUT,  VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT,  VX_TYPE_SCALAR, VX_PARAMETER_STATE_OPTIONAL},
    {VX_OUTPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_matrix_multiply_kernel = {
    VX_ADD_KERNEL_MATRIX_MULTIPLY,
    VX_ADD_KERNEL_NAME_MATRIX_MULTIPLY,
    vxMatrixMultiplyKernel,
    add_matrix_multiply_kernel_params, dimof(add_matrix_multiply_kernel_params),
    vxMatrixMultiplyInputValidator,
    vxMatrixMultiplyOutputValidator,
    NULL, NULL
};

