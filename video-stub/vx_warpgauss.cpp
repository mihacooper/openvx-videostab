#include "vx_warpgauss.h"

#define CHECK_SAVE_WARP_NODE(var, name) CHECK_SAVE_NODE(var, name, nodes_map)

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

vx_status WarpGaussGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output, vx_matrix* matrices,
                         vx_matrix& result_matr, WarpGaussParams& params, std::map<std::string, vx_node>& nodes_map)
{
    CHECK_NULL(context);
    //CHECK(dimof(matrices) != (params.gauss_size * 2));
    //CHECK(dimof(params.gauss_coeffs) != (params.gauss_size * 2 + 1));

    vx_size matr_num = params.gauss_size * 2;
    vx_uint32 width, height;
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
    vxQueryImage(input, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

    graph = vxCreateGraph(context);
    CHECK_NULL(graph);

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
    vx_scalar inter_s = vxCreateScalar(context, VX_TYPE_ENUM, &params.interpol);
    vx_matrix inv_matr = vxCreateMatrix(context, VX_TYPE_FLOAT32, 3, 3);
    CHECK_NULL(vxMatrixInvertNode(graph, prev_sum_matr, inv_matr));
    vx_node warp_node = vxWarpPerspectiveRGBNode(graph, input, inv_matr, inter_s, output);
    CHECK_SAVE_WARP_NODE(warp_node, "WarpPerspective");
    vx_border_mode_t border = {VX_BORDER_MODE_CONSTANT, 0};
    vxSetNodeAttribute(warp_node, VX_NODE_ATTRIBUTE_BORDER_MODE, &border, sizeof(border));
    CHECK_STATUS( vxVerifyGraph(graph) );
    return VX_SUCCESS;
}

/*** CVWarp ***
vxCVWarpNode(graph, (vx_image)vxGetReferenceFromDelay(m_Images, m_NumImages / 2), prev_sum_matr, m_OutImage);
 **************/
