#include "vx_pipelines.h"

#define CHECK_SAVE_OPT_NODE(var, name) CHECK_SAVE_NODE(var, name, nodes_map)

vx_status FindWarpGraph(vx_context context, vx_graph& graph,vx_image from_image, vx_image to_image,
                        vx_matrix matrix, FindWarpParams& params, std::map<std::string, vx_node>& nodes_map)
{
    CHECK_NULL(context);

    vx_uint32 width, height;
    vxQueryImage(from_image, VX_IMAGE_ATTRIBUTE_WIDTH,  &width,  sizeof(width));
    vxQueryImage(from_image, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

    /***    Internal params    ***/
    vx_uint32 corners_num = 100;
    vx_uint32 optflow_init_estimate = vx_false_e;
    /***    End of internal params    ***/

    /***    Create objects    ***/
               graph             = vxCreateGraph(context);
    vx_image   gray_image_1      = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_U8);
    vx_image   gray_image_2      = vxCreateVirtualImage(graph, width, height, VX_DF_IMAGE_U8);
    vx_scalar  fast_thresh_s     = vxCreateScalar(context, VX_TYPE_FLOAT32, &params.fast_thresh);
    vx_scalar  fast_num_corn_s   = vxCreateScalar(context, VX_TYPE_UINT32, &corners_num);
    vx_array   fast_found_corn_s = vxCreateArray(context, VX_TYPE_KEYPOINT, params.fast_max_corners);
    vx_array   optf_moved_corn_s = vxCreateArray(context, VX_TYPE_KEYPOINT, params.fast_max_corners);
    vx_scalar  optf_estimate_s   = vxCreateScalar(context, VX_TYPE_FLOAT32, &params.optflow_estimate);
    vx_scalar  optf_max_iter_s   = vxCreateScalar(context, VX_TYPE_UINT32, &params.optflow_max_iter);
    vx_scalar  optf_init_estim   = vxCreateScalar(context, VX_TYPE_BOOL, &optflow_init_estimate);
    vx_pyramid pyramid_1         = vxCreatePyramid(context, params.pyramid_level, params.pyramid_scale, width, height, VX_DF_IMAGE_U8);
    vx_pyramid pyramid_2         = vxCreatePyramid(context, params.pyramid_level, params.pyramid_scale, width, height, VX_DF_IMAGE_U8);
    /***      Check objects   ***/
    CHECK_NULL(graph);
    CHECK_NULL(gray_image_1);
    CHECK_NULL(gray_image_2);
    CHECK_NULL(fast_thresh_s);
    CHECK_NULL(fast_num_corn_s);
    CHECK_NULL(fast_found_corn_s);
    CHECK_NULL(optf_moved_corn_s);
    CHECK_NULL(pyramid_1);
    CHECK_NULL(pyramid_2);
    /***    End of objects    ***/

    vx_node node[7];
    node[0] = vxRGBtoGrayNode(graph, from_image, gray_image_1);
    node[1] = vxRGBtoGrayNode(graph, to_image, gray_image_2);
    node[2] = vxFastCornersNode(graph, gray_image_1, fast_thresh_s, vx_true_e, fast_found_corn_s, fast_num_corn_s);
    node[3] = vxGaussianPyramidNode(graph, gray_image_1, pyramid_1);
    node[4] = vxGaussianPyramidNode(graph, gray_image_2, pyramid_2);
    node[5] = vxOpticalFlowPyrLKNode(graph, pyramid_1, pyramid_2, fast_found_corn_s,
                    fast_found_corn_s, optf_moved_corn_s, params.optflow_term,
                    optf_estimate_s, optf_max_iter_s, optf_init_estim, params.optflow_wnd_size);
    node[6] = vxFindWarpNode(graph, fast_found_corn_s, optf_moved_corn_s, matrix);

    CHECK_SAVE_OPT_NODE( node[0], "RGBtoGray_old");
    CHECK_SAVE_OPT_NODE( node[1], "RGBtoGray_new");
    CHECK_SAVE_OPT_NODE( node[2], "FAST");

    CHECK_SAVE_OPT_NODE( node[3], "GaussianPyr_old");
    CHECK_SAVE_OPT_NODE( node[4], "GaussianPyr_new");
    CHECK_SAVE_OPT_NODE( node[5], "OptFlow");
    CHECK_SAVE_OPT_NODE( node[6], "FindWarp");

    CHECK_STATUS( vxVerifyGraph(graph) );
    return VX_SUCCESS;
}

/*** Harris sample ***
    vx_float32 harr_thresh = 1000.;
    vx_float32 harr_dist = 5.;
    vx_float32 harr_sens = 0.050000f;
    vx_scalar harr_thresh_s = vxCreateScalar(context, VX_TYPE_FLOAT32, &harr_thresh);
    vx_scalar harr_dist_s = vxCreateScalar(context, VX_TYPE_FLOAT32, &harr_dist);
    vx_scalar harr_sens_s = vxCreateScalar(context, VX_TYPE_FLOAT32, &harr_sens);
    CHECK_NULL(vxHarrisCornersNode(graph, gray_image_1, harr_thresh_s, harr_dist_s, harr_sens_s, 3, 3, fast_found_corn_s, fast_num_corn_s));
 *********************/

/*** CVOptFlow sample ***
CHECK_SAVE_OPT_NODE( vxCVOptFlowNode(graph, gray_image_1, gray_image_2, fast_found_corn_s, optf_moved_corn_s), "CV_OptFlow")
 ************************/
