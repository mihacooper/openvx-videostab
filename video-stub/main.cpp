#include <cstdio>
#include "VX/vx.h"
#include "vx_debug.h"
#include "vreader.h"
#include "add_kernels/add_kernels.h"

#define INIT_DEBUG(zones, num) { for(int i = 0; i < (num); i++) vx_set_debug_zone((zones)[i]); }
#define CHECK_NULL(var) { if( (var) == NULL ) { VX_PRINT(VX_ZONE_ERROR, "NULL reference of " #var "\n"); } }
#define CHECK_STATUS(var) { if( (var) != VX_SUCCESS ){ VX_PRINT(VX_ZONE_ERROR, #var "return bad status\n"); } }

vx_enum zones[] = {
//    VX_ZONE_ARRAY,
//    VX_ZONE_GRAPH,
    VX_ZONE_ERROR,
    VX_ZONE_WARNING,
//    VX_ZONE_INFO,
//    VX_ZONE_IMAGE,
    VX_ZONE_LOG
};

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
        printf("Use ./%s <video_name>\n", argv[0]);
        return 0;
    }

    VReader reader(argv[1]);
    INIT_DEBUG(zones, sizeof(zones) / sizeof(zones[0]));
    while(true)
    {
        vx_context context = vxCreateContext();
        CHECK_NULL(context);
        vx_graph graph = vxCreateGraph(context);
        CHECK_NULL(graph);

        vx_image image = NULL;
        if(!reader.ReadFrame(context, image)) break;
        CHECK_NULL(image);

        vx_uint32 width, height;
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
        vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
        vx_image gray = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_U8);
        vxRGBtoGrayNode(graph, image, gray);

        vx_float32 fast_thresh = 50.f;
        vx_uint32 corners_num = 10;
        vx_scalar fast_scalar = vxCreateScalar(context, VX_TYPE_FLOAT32, &fast_thresh);
        CHECK_NULL(fast_scalar);
        vx_scalar corners_scalar = vxCreateScalar(context, VX_TYPE_UINT32, &corners_num);
        CHECK_NULL(corners_scalar);
        vx_array corners = vxCreateArray(context, VX_TYPE_KEYPOINT, 100);
        CHECK_NULL(corners);

        CHECK_NULL(vxFastCornersNode(graph, gray, fast_scalar, vx_true_e, corners, corners_scalar));

        CHECK_STATUS(vxVerifyGraph(graph));
        CHECK_STATUS(vxProcessGraph(graph));

        CHECK_STATUS(vxAccessScalarValue(corners_scalar, &corners_num));
        VX_PRINT(VX_ZONE_LOG, "%d found corners\n", corners_num);
        vxReleaseContext(&context);
    }
    return 0;
}
