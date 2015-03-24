#include "vx_module.h"
#include "math.h"
#include "memory.h"
#include <cstdio>


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
    m_Matrices(NULL), m_OptFlowGraph(NULL), m_WarpGraph(NULL), m_ImageAdded(vx_false_e)
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
    m_NumImages = params.gauss_size * 2 + 2;
    m_NumMatr = params.gauss_size * 2 + 1;
    /******Internal params******/
    vx_uint32 corners_num = 100;
    vx_uint32 optflow_init_estimate = vx_false_e;
    vx_float32 sigma = params.gauss_size * 0.7;
    vx_float32 matr_coeffs[m_NumMatr];
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

    CHECK_NULL( vxRGBtoGrayNode(m_OptFlowGraph, (vx_image)vxGetReferenceFromDelay(m_Images, 1 ), gray_image_1) );
    CHECK_NULL( vxRGBtoGrayNode(m_OptFlowGraph, (vx_image)vxGetReferenceFromDelay(m_Images, 0 ), gray_image_2) );
    /*
    vx_float32 harr_thresh = 32768.;
    vx_float32 harr_dist = 1.1;
    vx_float32 harr_sens = 0.15;
    vx_scalar harr_thresh_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &harr_thresh);
    vx_scalar harr_dist_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &harr_dist);
    vx_scalar harr_sens_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &harr_sens);
    CHECK_NULL(vxHarrisCornersNode(m_OptFlowGraph, gray_image_1, harr_thresh_s, harr_dist_s, harr_sens_s, 3, 7, fast_found_corn_s, fast_num_corn_s));
    */
    CHECK_NULL( vxFastCornersNode(m_OptFlowGraph, gray_image_1, fast_thresh_s, vx_true_e, fast_found_corn_s, fast_num_corn_s) );

    CHECK_NULL( vxGaussianPyramidNode(m_OptFlowGraph, gray_image_1, pyramid_1) );
    CHECK_NULL( vxGaussianPyramidNode(m_OptFlowGraph, gray_image_2, pyramid_2) );
    CHECK_NULL( vxOpticalFlowPyrLKNode(m_OptFlowGraph, pyramid_1, pyramid_2, fast_found_corn_s,
                                       fast_found_corn_s, optf_moved_corn_s, params.optflow_term,
                                       optf_estimate_s, optf_max_iter_s, optf_init_estim, params.optflow_wnd_size) );
    CHECK_NULL( vxFindWarpNode(m_OptFlowGraph, fast_found_corn_s, optf_moved_corn_s, (vx_matrix)vxGetReferenceFromDelay(m_Matrices, 0 )) );

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
           continue;
        }
        int start = j < center ? j : center + 1;
        int end = j > center ? j : center;
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
        vx_scalar coeff_s = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &matr_coeffs[j]);
        vx_matrix next_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
        CHECK_NULL(vxMatrixAddNode(m_WarpGraph, prev_mul_matr, prev_sum_matr, coeff_s, next_matr));
        prev_sum_matr = next_matr;
    }
    vx_enum inter = VX_INTERPOLATION_TYPE_NEAREST_NEIGHBOR;
    vx_scalar inter_s = vxCreateScalar(m_Context, VX_TYPE_ENUM, &inter);
    vx_matrix inv_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    CHECK_NULL(vxMatrixInvertNode(m_WarpGraph, prev_sum_matr, inv_matr));
    vx_node warp_node = vxWarpPerspectiveRGBNode(m_WarpGraph, (vx_image)vxGetReferenceFromDelay(m_Images, m_NumImages / 2),
                                              inv_matr, inter_s, m_OutImage);
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
        ret = m_OutImage;
        m_CurImageId--;
    }
    vxAgeDelay(m_Images);
    vxAgeDelay(m_Matrices);
    m_ImageAdded = vx_false_e;
    return ret;
}

