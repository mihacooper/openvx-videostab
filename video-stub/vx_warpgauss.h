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

vx_status WarpGaussGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output, vx_matrix* matrices,
                         vx_matrix& result_matr, WarpGaussParams& params, std::map<std::string, vx_node>& nodes_map);

#endif // VX_WARPGAUSS_H
