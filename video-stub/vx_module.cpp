#include "vx_module.h"
#include "math.h"
#include "memory.h"
#include <cstdio>
#include <string>
#include <ctime>

#define INIT_DEBUG(zones, num) \
    { \
        for(int i = 0; i < (num); i++) \
            vx_set_debug_zone((zones)[i]); \
    }

#define CHECK_NULL(var) \
    { \
        if( (var) == NULL ) \
        {\
            VX_PRINT(VX_ZONE_ERROR, "NULL reference of " #var "\n"); \
            return VX_FAILURE; \
        } \
    }

#define CHECK_SAVE_NODE(var, name, cont) \
    { \
        vx_node node = (var);\
        CHECK_NULL(node);\
        (cont).insert(std::make_pair(name, node));\
    }

#define CHECK_SAVE_OPT_NODE(var, name) CHECK_SAVE_NODE(var, name, m_OptFlowNodes)
#define CHECK_SAVE_WARP_NODE(var, name) CHECK_SAVE_NODE(var, name, m_WarpNodes)

#define CHECK_STATUS(var) \
    { \
        if( (var) != VX_SUCCESS ) \
        { \
            VX_PRINT(VX_ZONE_ERROR, #var "return bad status\n"); \
            return VX_FAILURE; \
        } \
    }

VXVideoStab::VXVideoStab() :
    m_CurImageId(0), m_NumImages(0), m_Images(NULL),
    m_Matrices(NULL), m_OptFlowGraph(NULL), m_WarpGraph(NULL),
    m_ImageAdded(vx_false_e), m_MaxArea(0)
{
    m_Context = vxCreateContext();
    if(m_Context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create VX context. Stop.");
    }
}

VXVideoStab::~VXVideoStab()
{
    vxReleaseContext(&m_Context);
}

vx_status VXVideoStab::EnableDebug(const std::initializer_list<vx_enum>& zones)
{
    for(auto i = zones.begin(); i != zones.end(); i++) \
        vx_set_debug_zone(*i);
}

vx_status VXVideoStab::CreatePipeline(const vx_uint32 width, const vx_uint32 height, VideoStabParams& params)
{
    if(m_Context == NULL)
        return VX_FAILURE;

    int i, j;
    m_NumImages = params.gauss_size * 2 + 1;
    m_NumMatr = params.gauss_size * 2;
    /******Internal params******/
    vx_uint32 corners_num = 100;
    vx_uint32 optflow_init_estimate = vx_false_e;
    vx_float32 sigma = params.gauss_size * 0.7;
    vx_float32 matr_coeffs[m_NumImages];
    for (i = -params.gauss_size; i <= params.gauss_size; ++i)
        matr_coeffs[i + params.gauss_size] = ( exp(-i * i / (2.f * sigma * sigma)) );
    vx_float32 sum = 0.;
    for (i = 0; i < dimof(matr_coeffs); ++i)
        sum += matr_coeffs[i];
    sum = 1. / sum;
    for (i = 0; i < dimof(matr_coeffs); ++i)
    {
        matr_coeffs[i] *= sum;
    }
    /******End of params******/

    m_OptFlowGraph = vxCreateGraph(m_Context);
    CHECK_NULL(m_OptFlowGraph);

    /***     Create images   ***/
    vx_image tmp_image = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    m_Images = vxCreateDelay(m_Context, (vx_reference)tmp_image, m_NumImages);
    vx_image gray_image_1 = vxCreateVirtualImage(m_OptFlowGraph, width, height, VX_DF_IMAGE_U8);
    vx_image gray_image_2 = vxCreateVirtualImage(m_OptFlowGraph, width, height, VX_DF_IMAGE_U8);
    /***     Check images    ***/
    CHECK_NULL(m_Images);
    CHECK_NULL(gray_image_1);
    CHECK_NULL(gray_image_2);
    /***    End of images    ***/

    /***    Create objects    ***/
    vx_matrix tmp_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    m_Matrices = vxCreateDelay(m_Context, (vx_reference)tmp_matr, m_NumMatr);
    vx_scalar  fast_thresh_s     = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &params.fast_thresh);
    vx_scalar  fast_num_corn_s   = vxCreateScalar(m_Context, VX_TYPE_UINT32, &corners_num);
    vx_array   fast_found_corn_s = vxCreateArray(m_Context, VX_TYPE_KEYPOINT, params.fast_max_corners);
    vx_array   optf_moved_corn_s = vxCreateArray(m_Context, VX_TYPE_KEYPOINT, params.fast_max_corners);
    vx_scalar  optf_estimate_s   = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &params.optflow_estimate);
    vx_scalar  optf_max_iter_s   = vxCreateScalar(m_Context, VX_TYPE_UINT32, &params.optflow_max_iter);
    vx_scalar  optf_init_estim   = vxCreateScalar(m_Context, VX_TYPE_BOOL, &optflow_init_estimate);
    vx_pyramid pyramid_1         = vxCreatePyramid(m_Context, params.pyramid_level, params.pyramid_scale, width, height, VX_DF_IMAGE_U8);
    vx_pyramid pyramid_2         = vxCreatePyramid(m_Context, params.pyramid_level, params.pyramid_scale, width, height, VX_DF_IMAGE_U8);
    /***      Check objects   ***/
    CHECK_NULL(fast_thresh_s);
    CHECK_NULL(fast_num_corn_s);
    CHECK_NULL(fast_found_corn_s);
    CHECK_NULL(optf_moved_corn_s);
    CHECK_NULL(pyramid_1);
    CHECK_NULL(pyramid_2);
    /***    End of objects    ***/

    CHECK_SAVE_OPT_NODE( vxRGBtoGrayNode(m_OptFlowGraph, (vx_image)vxGetReferenceFromDelay(m_Images, 1 ), gray_image_1), "RGBtoGray_old");
    CHECK_SAVE_OPT_NODE( vxRGBtoGrayNode(m_OptFlowGraph, (vx_image)vxGetReferenceFromDelay(m_Images, 0 ), gray_image_2), "RGBtoGray_new");
    /*
    vx_float32 harr_thresh = 32768.;
    vx_float32 harr_dist = 1.1;
    vx_float32 harr_sens = 0.15;
    vx_scalar harr_thresh_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &harr_thresh);
    vx_scalar harr_dist_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &harr_dist);
    vx_scalar harr_sens_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &harr_sens);
    CHECK_NULL(vxHarrisCornersNode(m_OptFlowGraph, gray_image_1, harr_thresh_s, harr_dist_s, harr_sens_s, 3, 7, fast_found_corn_s, fast_num_corn_s));
    */
    CHECK_SAVE_OPT_NODE( vxFastCornersNode(m_OptFlowGraph, gray_image_1, fast_thresh_s, vx_true_e, fast_found_corn_s, fast_num_corn_s), "FAST");

    CHECK_SAVE_OPT_NODE( vxGaussianPyramidNode(m_OptFlowGraph, gray_image_1, pyramid_1), "GaussianPyr_old");
    CHECK_SAVE_OPT_NODE( vxGaussianPyramidNode(m_OptFlowGraph, gray_image_2, pyramid_2), "GaussianPyr_new");
    CHECK_SAVE_OPT_NODE( vxOpticalFlowPyrLKNode(m_OptFlowGraph, pyramid_1, pyramid_2, fast_found_corn_s,
                                       fast_found_corn_s, optf_moved_corn_s, params.optflow_term,
                                       optf_estimate_s, optf_max_iter_s, optf_init_estim, params.optflow_wnd_size), "OptFlow");
    CHECK_SAVE_OPT_NODE( vxFindWarpNode(m_OptFlowGraph, fast_found_corn_s, optf_moved_corn_s, (vx_matrix)vxGetReferenceFromDelay(m_Matrices, 0 )), "FindWarp");

    CHECK_STATUS( vxVerifyGraph(m_OptFlowGraph) );

    /*** Warp Graph***/
    m_WarpGraph = vxCreateGraph(m_Context);
    CHECK_NULL(m_WarpGraph);
    m_OutImage = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    vx_matrix prev_sum_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    vx_float32 matr_ptr[9];
    vxAccessMatrix(prev_sum_matr, matr_ptr);
    memset(matr_ptr, 0, sizeof(vx_float32) * 9);
    vxCommitMatrix(prev_sum_matr, matr_ptr);

    int center = m_NumMatr / 2;
    for(j = 0; j < m_NumMatr; j++)
    {
        if( j == center)
        {
           vx_matrix eye_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
           vx_float32 matr_ptr[9];
           vxAccessMatrix(eye_matr, matr_ptr);
           memset(matr_ptr, 0, sizeof(vx_float32) * 9);
           matr_ptr[0] = matr_ptr[4] = matr_ptr[8] = 1.;
           vxCommitMatrix(eye_matr, matr_ptr);

           vx_scalar coeff_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &matr_coeffs[j]);
           vx_matrix next_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
           CHECK_NULL(vxMatrixAddNode(m_WarpGraph, eye_matr, prev_sum_matr, coeff_s, next_matr));
           prev_sum_matr = next_matr;
        }
        int start = j < center ? j : center;
        int end = j >= center ? j : center - 1;
        int coef_ind = j >= center ? j + 1 : j;
        vx_matrix prev_mul_matr = (vx_matrix)vxGetReferenceFromDelay(m_Matrices, end);
        for(i = end - 1; i >= start; i--)
        {
            vx_matrix next_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
            CHECK_NULL(vxMatrixMultiplyNode(m_WarpGraph, prev_mul_matr, (vx_matrix)vxGetReferenceFromDelay(m_Matrices, i ), NULL, next_matr));
            prev_mul_matr = next_matr;
        }
        if(j > center)
        {
           vx_matrix inv_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
           CHECK_NULL(vxMatrixInvertNode(m_WarpGraph, prev_mul_matr, inv_matr));
           prev_mul_matr = inv_matr;
        }
        vx_scalar coeff_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &matr_coeffs[coef_ind]);
        vx_matrix next_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
        CHECK_NULL(vxMatrixAddNode(m_WarpGraph, prev_mul_matr, prev_sum_matr, coeff_s, next_matr));
        prev_sum_matr = next_matr;
    }
    vx_enum inter = VX_INTERPOLATION_TYPE_NEAREST_NEIGHBOR;
    vx_scalar inter_s = vxCreateScalar(m_Context, VX_TYPE_ENUM, &inter);
    vx_matrix inv_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    m_ResMatr = prev_sum_matr;
    CHECK_NULL(vxMatrixInvertNode(m_WarpGraph, prev_sum_matr, inv_matr));
    vx_node warp_node = vxWarpPerspectiveRGBNode(m_WarpGraph, (vx_image)vxGetReferenceFromDelay(m_Images, m_NumImages / 2),
                                              inv_matr, inter_s, m_OutImage);
    CHECK_SAVE_WARP_NODE(warp_node, "WarpPerspective");
    vx_border_mode_t border = {VX_BORDER_MODE_CONSTANT, 0};
    vxSetNodeAttribute(warp_node, VX_NODE_ATTRIBUTE_BORDER_MODE, &border, sizeof(border));
    CHECK_STATUS( vxVerifyGraph(m_WarpGraph) );
    return VX_SUCCESS;
    /*****************/
}

vx_image VXVideoStab::NewImage()
{
    if(m_Images == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Pipeline wasn't created!\n");
        return NULL;
    }

    if(m_CurImageId < m_NumImages)
    {
        m_ImageAdded = vx_true_e;
        vx_image ret = (vx_image)vxGetReferenceFromDelay(m_Images, 0);
        m_CurImageId++;
        return ret;
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't store more image, need calculate!\n");
        return NULL;
    }
}

vx_image VXVideoStab::CopyImage(vx_image input)
{
    vx_rectangle_t rect;
    vx_imagepatch_addressing_t addr1, addr2;
    void* buff1 = NULL, *buff2 = NULL;

    vxGetValidRegionImage(input, &rect);
    vxAccessImagePatch(input, &rect, 0, &addr1, &buff1, VX_READ_AND_WRITE);
    vx_image output = vxCreateImage(m_Context, addr1.dim_x, addr1.dim_y, VX_DF_IMAGE_RGB);
    vxAccessImagePatch(output, &rect, 0, &addr2, &buff2, VX_READ_AND_WRITE);
    memcpy(buff2, buff1, addr1.dim_x * addr1.dim_y * 3);
    vxCommitImagePatch(input, NULL, 0, &addr1, buff1);
    vxCommitImagePatch(output, NULL, 0, &addr2, buff2);
    return output;
}

void VXVideoStab::TransformPerspective(vx_uint32& x, vx_uint32& y, vx_int32& tx, vx_int32& ty, const vx_float32 m[])
{
    vx_float32 z = x * m[6] + y * m[7] + m[8];
    tx = (x * m[0] + y * m[1] + m[2]) / z;
    ty = (x * m[3] + y * m[4] + m[5]) / z;
}

struct Point
{
    vx_int32 x,y;
};

vx_int32 VXVideoStab::DefineValidRect(vx_image image, vx_matrix matrix)
{
    vx_rectangle_t rect = {0, 0, 0, 0}, mrect;
    vx_float32 matr[9];
    Point pnt[4];
    vx_uint32 width, height;
    vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH,  &width, sizeof(width));
    vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));
    rect.end_x = width;
    rect.end_y = height;
    memcpy(&mrect, &rect, sizeof(mrect));

    vxAccessMatrix(matrix, matr);

    TransformPerspective(rect.start_x, rect.start_y, pnt[0].x, pnt[0].y, matr);
    TransformPerspective(rect.end_x,   rect.start_y, pnt[1].x, pnt[1].y, matr);
    TransformPerspective(rect.end_x,   rect.end_y,   pnt[2].x, pnt[2].y, matr);
    TransformPerspective(rect.start_x, rect.end_y,   pnt[3].x, pnt[3].y, matr);

    vx_int32 max_area = 0;
    #define AREA_X(x) ((x)*(x) * height)/width
    #define AREA_Y(y) ((y)*(y) * width)/height
    #define max(a, b) (a) > (b) ? (a) : (b);

    if(pnt[0].x > 0 && pnt[0].y > 0)
    {
        max_area = max(AREA_X(pnt[0].x), max_area);
        max_area = max(AREA_Y(pnt[0].y), max_area);
    }
    if(pnt[1].x < width && pnt[1].y > 0)
    {
        max_area = max(AREA_X(width - pnt[1].x), max_area);
        max_area = max(AREA_Y(pnt[1].y), max_area);
    }
    if(pnt[2].x < width && pnt[2].y < height)
    {
        max_area = max(AREA_X(width - pnt[2].x), max_area);
        max_area = max(AREA_Y(height - pnt[2].y), max_area);
    }
    if(pnt[3].x > 0 && pnt[3].y < height)
    {
        max_area = max(AREA_X(pnt[3].x), max_area);
        max_area = max(AREA_Y(height - pnt[3].y), max_area);
    }


    vxCommitMatrix(matrix, matr);
    return max_area;
}

vx_status VXVideoStab::EnableCuting(vx_uint32 width, vx_uint32 height)
{
   vx_float32 prop = (vx_float32)(width) / height;
   vx_uint32 sub_height = sqrt((vx_float32)m_MaxArea / prop);
   vx_uint32 sub_width  = sub_height * prop;
   vx_rectangle_t rect;
   rect.start_x = sub_width;
   rect.start_y = sub_height;
   rect.end_x   = width - sub_width;
   rect.end_y   = height - sub_height;

   m_CutGraph = vxCreateGraph(m_Context);
   vx_image chan[3], scaled[3];
   vx_enum chanels[] = {VX_CHANNEL_R, VX_CHANNEL_G, VX_CHANNEL_B};
   vx_scalar sx = vxCreateScalar(m_Context, VX_TYPE_UINT32, &rect.start_x);
   vx_scalar sy = vxCreateScalar(m_Context, VX_TYPE_UINT32, &rect.start_y);
   vx_scalar ex = vxCreateScalar(m_Context, VX_TYPE_UINT32, &rect.end_x);
   vx_scalar ey = vxCreateScalar(m_Context, VX_TYPE_UINT32, &rect.end_y);
   vx_image cuted = vxCreateVirtualImage(m_CutGraph, rect.end_x - rect.start_x, rect.end_y - rect.start_y, VX_DF_IMAGE_RGB);
   vx_image fakeCutInput = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
   m_CutedImage = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);

   m_CutNode = vxCutNode(m_CutGraph, fakeCutInput , sx, sy, ex, ey, cuted);
   for(int i = 0; i < 3; i++)
   {
       chan[i] = vxCreateVirtualImage(m_CutGraph, rect.end_x - rect.start_x, rect.end_y - rect.start_y, VX_DF_IMAGE_U8);
       scaled[i] = vxCreateVirtualImage(m_CutGraph, width, height, VX_DF_IMAGE_U8);
       vxChannelExtractNode(m_CutGraph, cuted, chanels[i], chan[i]);
       vxScaleImageNode(m_CutGraph, chan[i], scaled[i], VX_INTERPOLATION_TYPE_NEAREST_NEIGHBOR);
   }
   vxChannelCombineNode(m_CutGraph, scaled[0], scaled[1], scaled[2], NULL, m_CutedImage);

   CHECK_STATUS(vxVerifyGraph(m_CutGraph));
   return VX_SUCCESS;
}

vx_image VXVideoStab::CutImage(vx_image input)
{
    vxSetParameterByIndex(m_CutNode, 0, (vx_reference)input);
    if(vxProcessGraph(m_CutGraph) != VX_SUCCESS)
    {
       printf("Error: process graph failure");
    }
    return m_CutedImage;
}

vx_image VXVideoStab::Calculate()
{
    if(!m_ImageAdded)
    {
        VX_PRINT(VX_ZONE_WARNING, "Add new image first!\n");
        return NULL;
    }

    if(m_CurImageId > 1)
    {
        if(vxProcessGraph(m_OptFlowGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Optical flow graph process error!\n");
            return NULL;
        }
    }
    vx_image ret = NULL;
    if(m_CurImageId == m_NumImages)
    {
        if(vxProcessGraph(m_WarpGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Warp graph process error!\n");
            return NULL;
        }
        ret = CopyImage(m_OutImage);
        vx_int32 area = DefineValidRect(m_OutImage, m_ResMatr);
        m_MaxArea = max(m_MaxArea, area);
        m_CurImageId--;
    }
    vxAgeDelay(m_Images);
    vxAgeDelay(m_Matrices);
    m_ImageAdded = vx_false_e;
    return ret;
}

#define BILLION (1000000000)

double VXVideoStab::AvgPerf(vx_perf_t& perf)
{
    return ((double)perf.avg) / BILLION;
}

void VXVideoStab::NodesPerf(std::map<std::string, vx_node>& nodes)
{
    vx_perf_t perf;
    for(auto it = nodes.begin(); it != nodes.end(); it++)
    {
        vxQueryNode(it->second, VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf));
        printf("\t%s = %lf\n", it->first.c_str(), AvgPerf(perf));
    }
}

void VXVideoStab::PrintPerf()
{
    vx_perf_t perf;
    vxQueryGraph(m_OptFlowGraph, VX_GRAPH_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf));
    printf("OptFlow graph(%lf):\n", AvgPerf(perf));
    for(auto it = m_OptFlowNodes.begin(); it != m_OptFlowNodes.end(); it++)
    {
        vxQueryNode(it->second, VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf));
        printf("\t%s = %lf\n", it->first.c_str(), AvgPerf(perf));
    }
    printf("Warp graph:\n");
    for(auto it = m_WarpNodes.begin(); it != m_WarpNodes.end(); it++)
    {
        vxQueryNode(it->second, VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf));
        printf("\t%s = %lf\n", it->first.c_str(), AvgPerf(perf));
    }
}

