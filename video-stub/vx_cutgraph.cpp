#include "vx_pipelines.h"

vx_status CutGraph(vx_context context, vx_graph& graph, vx_image input, vx_image output, vx_uint32 width, vx_uint32 height, vx_float32 scale)
{
    vx_rectangle_t rect;
    vx_uint32 sub_height = (height * (1 - scale)) / 2.;
    vx_uint32 sub_width  = (width * (1 - scale)) / 2.;
    rect.start_x = sub_width;
    rect.start_y = sub_height;
    rect.end_x   = width - sub_width;
    rect.end_y   = height - sub_height;

    graph = vxCreateGraph(context);
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

    CHECK_STATUS(vxVerifyGraph(graph));
    return VX_SUCCESS;
}

