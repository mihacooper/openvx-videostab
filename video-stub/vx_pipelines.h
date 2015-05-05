#ifndef VX_WARPGAUSS_H
#define VX_WARPGAUSS_H

#include "vx_common.h"
#include "map"
#include "string"

struct  WarpGaussParams
{
    vx_float32* gauss_coeffs;
    vx_uint32   gauss_size;
    vx_enum     interpol;
};

vx_status MatrixGaussGraph(vx_context context, vx_graph& graph, vx_uint32 width, vx_uint32 height, vx_matrix* matrices,
                         vx_matrix& result_matr, WarpGaussParams& params, std::map<std::string, vx_node>& nodes_map);

vx_status WarpGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output, vx_matrix matrix,
                         WarpGaussParams& params, std::map<std::string, vx_node>& nodes_map);

struct  FindWarpParams
{
    /*    FAST9    */
    vx_float32 fast_thresh;
    vx_uint32  fast_max_corners;
    /***************/

    /* GaussianPyramid */
    vx_float32 pyramid_scale;
    vx_size    pyramid_level;
    /*******************/

    /* OpticalFlow */
    vx_size    optflow_wnd_size;
    vx_enum    optflow_term;
    vx_float32 optflow_estimate;
    vx_uint32  optflow_max_iter;
    /*******************/
};

vx_status FindWarpGraph(vx_context context, vx_graph& graph,vx_image from_image, vx_image to_image,
                        vx_matrix matrix, FindWarpParams& params, std::map<std::string, vx_node>& nodes_map);

vx_status CutGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output, vx_uint32 width, vx_uint32 height, vx_float32 scale);

vx_status ModifyMatrix(vx_matrix matr, vx_uint32 width, vx_uint32 height, vx_float32 scale);

#endif // VX_WARPGAUSS_H
