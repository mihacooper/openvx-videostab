#include "add_kernels.h"

vx_node vxRGBtoGrayNode(vx_graph graph, vx_image input, vx_image output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input,
            (vx_reference)output,
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_RGB_TO_GRAY,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxFindWarpNode(vx_graph graph, vx_array def_pnts, vx_array moved_pnts, vx_matrix matr)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)def_pnts,
            (vx_reference)moved_pnts,
            (vx_reference)matr,
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_FIND_WARP,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxWarpPerspectiveRGBNode(vx_graph graph, vx_image input, vx_matrix matr, vx_scalar inter, vx_image output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input,
            (vx_reference)matr,
            (vx_reference)inter,
            (vx_reference)output,
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_WARP_PERSPECTIVE_RGB,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxMatrixMultiplyNode(vx_graph graph, vx_matrix input1, vx_matrix input2, vx_scalar coeff, vx_matrix output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input1,
            (vx_reference)input2,
            (vx_reference)coeff,
            (vx_reference)output
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_MATRIX_MULTIPLY,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxMatrixAddNode(vx_graph graph, vx_matrix input1, vx_matrix input2, vx_scalar coeff, vx_matrix output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input1,
            (vx_reference)input2,
            (vx_reference)coeff,
            (vx_reference)output
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_MATRIX_ADD,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxMatrixInvertNode(vx_graph graph, vx_matrix input, vx_matrix output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input,
            (vx_reference)output
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_MATRIX_INVERT,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxCutNode(vx_graph graph, vx_image input, vx_scalar left, vx_scalar right, vx_scalar top, vx_scalar bottom, vx_image output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input,
            (vx_reference)left,
            (vx_reference)right,
            (vx_reference)top,
            (vx_reference)bottom,
            (vx_reference)output
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_CUT,
                                       params,
                                       dimof(params));
    }
    return node;
}

vx_node vxMatrixModifyNode(vx_graph graph, vx_matrix input, vx_scalar width, vx_scalar height, vx_scalar scale, vx_matrix output)
{
    vx_node node = 0;
    vx_context context = vxGetContext((vx_reference)graph);
    vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
    if (status == VX_SUCCESS)
    {
        vx_reference params[] = {
            (vx_reference)input,
            (vx_reference)width,
            (vx_reference)height,
            (vx_reference)scale,
            (vx_reference)output
        };
        node = vxCreateNodeByStructure(graph,
                                       VX_ADD_KERNEL_MATRIX_MODIFY,
                                       params,
                                       dimof(params));
    }
    return node;
}

