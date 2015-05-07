#include "vx_pipelines.h"

static vx_matrix CreateZeroMatrix(vx_context context)
{
    vx_matrix matrix = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
    vx_float32 matr_ptr[9];
    vxAccessMatrix(matrix, matr_ptr);
    memset(matr_ptr, 0, sizeof(vx_float32) * 9);
    vxCommitMatrix(matrix, matr_ptr);
    return matrix;
}

static vx_matrix CreateEyeMatrix(vx_context context)
{
    vx_matrix matrix = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
    vx_float32 matr_ptr[9];
    vxAccessMatrix(matrix, matr_ptr);
    memset(matr_ptr, 0, sizeof(vx_float32) * 9);
    matr_ptr[0] = matr_ptr[4] = matr_ptr[8] = 1.;
    vxCommitMatrix(matrix, matr_ptr);
    return matrix;
}

static vx_status CreateMatrixGauss(vx_context context, vx_graph& graph, vx_matrix* matrices,
                         vx_matrix result_matr, WarpGaussParams& params)
{
    vx_size matr_num = params.gauss_size * 2;
    vx_matrix prev_sum_matr = CreateZeroMatrix(context);
    vx_size center = matr_num / 2;

    for(int j = 0; j < matr_num; j++)
    {
        if( j == center)
        {
           vx_matrix eye_matr = CreateEyeMatrix(context);
           vx_scalar coeff_s = vxCreateScalar(context, VX_TYPE_FLOAT32, &params.gauss_coeffs[j]);
           vx_matrix next_matr = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
           CHECK_NULL(vxMatrixAddNode(graph, eye_matr, prev_sum_matr, coeff_s, next_matr));
           prev_sum_matr = next_matr;
        }

        int start = j < center ? j : center;
        int end = j >= center ? j : center - 1;
        int coef_ind = j >= center ? j + 1 : j;

        vx_matrix prev_mul_matr = matrices[end];
        for(int i = end - 1; i >= start; i--)
        {
            vx_matrix next_matr = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
            CHECK_NULL(vxMatrixMultiplyNode(graph, prev_mul_matr, matrices[i], NULL, next_matr));
            prev_mul_matr = next_matr;
        }

        if(j >= center)
        {
           vx_matrix inv_matr = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
           CHECK_NULL(vxMatrixInvertNode(graph, prev_mul_matr, inv_matr));
           prev_mul_matr = inv_matr;
        }

        vx_scalar coeff_s = vxCreateScalar(context, VX_TYPE_FLOAT32, &params.gauss_coeffs[coef_ind]);
        vx_matrix next_matr;
        if( j == (matr_num - 1))
            next_matr = result_matr;
        else
            next_matr = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);

        CHECK_NULL(vxMatrixAddNode(graph, prev_mul_matr, prev_sum_matr, coeff_s, next_matr));
        prev_sum_matr = next_matr;
    }
    return VX_SUCCESS;
}

static vx_status WarpImage(vx_context context, vx_graph graph, vx_image input, vx_image output, vx_matrix matrix,
                         WarpGaussParams& params)
{
    vx_uint32 width, height;
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

    vx_scalar s_width  = vxCreateScalar(context, VX_TYPE_UINT32,  &width);
    vx_scalar s_height = vxCreateScalar(context, VX_TYPE_UINT32,  &height);
    vx_scalar s_scale  = vxCreateScalar(context, VX_TYPE_FLOAT32, &params.scale);
    vx_matrix mod_matr = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
    vxMatrixModifyNode(graph, matrix, s_width, s_height, s_scale, mod_matr);
    vx_scalar inter_s   = vxCreateScalar(context, VX_TYPE_ENUM, &params.interpol);
    vx_matrix inv_matr  = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
    vx_node invert_node = vxMatrixInvertNode(graph, mod_matr, inv_matr);
    vx_node warp_node   = vxWarpPerspectiveRGBNode(graph, input, inv_matr, inter_s, output);
    CHECK_NULL(warp_node);
    CHECK_NULL(invert_node);

    vx_border_mode_t border = {VX_BORDER_MODE_CONSTANT, 0};
    vxSetNodeAttribute(warp_node, VX_NODE_ATTRIBUTE_BORDER_MODE, &border, sizeof(border));
    return VX_SUCCESS;
}

static vx_status CutImage(vx_context context, vx_graph graph, vx_image input, vx_image output, WarpGaussParams& params)
{
    vx_rectangle_t rect;
    vx_uint32 width, height;
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

    vx_uint32 sub_height = (height * (1. - params.scale)) / 2.;
    vx_uint32 sub_width  = (width * (1. - params.scale)) / 2.;
    rect.start_x = sub_width;
    rect.start_y = sub_height;
    rect.end_x   = width - sub_width;
    rect.end_y   = height - sub_height;

    vx_image chan[3], scaled[3];
    vx_enum chanels[] = {VX_CHANNEL_R, VX_CHANNEL_G, VX_CHANNEL_B};
    vx_scalar sx = vxCreateScalar(context, VX_TYPE_UINT32, &rect.start_x);
    vx_scalar sy = vxCreateScalar(context, VX_TYPE_UINT32, &rect.start_y);
    vx_scalar ex = vxCreateScalar(context, VX_TYPE_UINT32, &rect.end_x);
    vx_scalar ey = vxCreateScalar(context, VX_TYPE_UINT32, &rect.end_y);
    vx_image cuted = vxCreateVirtualImage(graph, rect.end_x - rect.start_x, rect.end_y - rect.start_y, VX_DF_IMAGE_RGB);

    vxCutNode(graph, input , sx, sy, ex, ey, cuted);
    for(int i = 0; i < 3; i++)
    {
        chan[i] = vxCreateVirtualImage(graph, rect.end_x - rect.start_x, rect.end_y - rect.start_y, VX_DF_IMAGE_U8);
        scaled[i] = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_U8);
        vxChannelExtractNode(graph, cuted, chanels[i], chan[i]);
        vxScaleImageNode(graph, chan[i], scaled[i], VX_INTERPOLATION_TYPE_NEAREST_NEIGHBOR);
    }
    vxChannelCombineNode(graph, scaled[0], scaled[1], scaled[2], NULL, output);
    return VX_SUCCESS;
}


vx_status WarpGaussAndCutGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output,
                         vx_matrix* matrices, WarpGaussParams& params)
{
    vx_status status = VX_SUCCESS;
    graph = vxCreateGraph(context);
    CHECK_NULL(graph);

    vx_uint32 width, height;
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
    vx_matrix warp_matr    = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
    vx_image  warped_image = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_RGB);

    status = CreateMatrixGauss(context, graph, matrices, warp_matr, params);
    CHECK_STATUS(status);
    status = WarpImage(context, graph, input, warped_image, warp_matr, params);
    CHECK_STATUS(status);
    status = CutImage(context, graph, warped_image, output, params);
    CHECK_STATUS(status);
    return status;
}

/*** CVWarp ***
vxCVWarpNode(graph, (vx_image)vxGetReferenceFromDelay(m_Images, m_NumImages / 2), prev_sum_matr, m_OutImage);
 **************/
