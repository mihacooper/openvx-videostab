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
    vx_float32  scale;
};

vx_status WarpGaussAndCutGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output,
                         vx_matrix* matrices, WarpGaussParams& params);

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
                        vx_matrix matrix, FindWarpParams& params);

#endif // VX_WARPGAUSS_H
