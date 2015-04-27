#ifndef VX_FINDWARP_MODULE_H
#define VX_FINDWARP_MODULE_H

#include "vx_common.h"
#include "map"
#include "string"

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

#endif // VX_FINDWARP_MODULE_H
