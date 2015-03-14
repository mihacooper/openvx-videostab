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

inline vx_int32 min(vx_int32 left, vx_int32 right)
{
    return left < right ? left : right;
}

inline vx_int32 max(vx_int32 left, vx_int32 right)
{
    return left > right ? left : right;
}

VXVideoStub::VXVideoStub() :
    m_CurImageId(0), m_NumImages(0), m_Images(NULL), m_Matrices(NULL), m_OptFlowGraph(NULL)
{
    m_Context = vxCreateContext();
    if(m_Context == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create VX context. Stop.");
    }
}

VXVideoStub::~VXVideoStub()
{
    vxReleaseContext(&m_Context);
}

vx_status VXVideoStub::EnableDebug(const std::initializer_list<vx_enum>& zones)
{
    for(auto i = zones.begin(); i != zones.end(); i++) \
        vx_set_debug_zone(*i);
}

#define MAX_PYRAMID_LEVELS 10
#define MAX_FOUNDED_CORNERS 200

vx_status VXVideoStub::CreatePipeline(const vx_uint32 width, const vx_uint32 height, const vx_uint32 gauss_size)
{
    if(m_Context == NULL)
        return VX_FAILURE;

    m_NumImages = gauss_size;
    /*****FAST9 params*****/
    vx_float32 fast_thresh = 50.f; // threshold parameter of FAST9
    vx_uint32  corners_num = 10;    // tmp value for init scalar
    /*****OptFlow params*****/
    vx_size optflow_wnd_size = 5;
    vx_float32 pyramid_scale = VX_SCALE_PYRAMID_HALF;
    vx_size    pyramid_level = min(
                floor(log(vx_float32(optflow_wnd_size) / vx_float32(width)) / log(pyramid_scale)),
                floor(log(vx_float32(optflow_wnd_size) / vx_float32(height)) / log(pyramid_scale))
                );
    pyramid_level = max(1, min(pyramid_level, MAX_PYRAMID_LEVELS));
    vx_enum optflow_term = VX_TERM_CRITERIA_BOTH;
    vx_float32 optflow_estimate = 1;
    vx_uint32 optflow_max_iter = 100;
    vx_uint32 optflow_init_estimate = vx_false_e;
    /******End of params******/

    m_OptFlowGraph   = vxCreateGraph(m_Context);
    CHECK_NULL(m_OptFlowGraph);

    /***     Create images   ***/
    vx_image tmp_image = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    m_Images = vxCreateDelay(m_Context, (vx_reference)tmp_image, gauss_size);
    vx_image gray_image_1 = vxCreateVirtualImage(m_OptFlowGraph, width, height, VX_DF_IMAGE_U8);
    vx_image gray_image_2 = vxCreateVirtualImage(m_OptFlowGraph, width, height, VX_DF_IMAGE_U8);
    /***     Check images    ***/
    CHECK_NULL(m_Images);
    CHECK_NULL(gray_image_1);
    CHECK_NULL(gray_image_2);
    /***    End of images    ***/

    /***    Create objects    ***/
    vx_scalar  fast_thresh_s     = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &fast_thresh);
    vx_scalar  fast_num_corn_s   = vxCreateScalar(m_Context, VX_TYPE_UINT32, &corners_num);
    vx_array   fast_found_corn_s = vxCreateArray(m_Context, VX_TYPE_KEYPOINT, MAX_FOUNDED_CORNERS);
    vx_array   optf_moved_corn_s = vxCreateArray(m_Context, VX_TYPE_KEYPOINT, MAX_FOUNDED_CORNERS);
    vx_array   optf_extim_corn_s = vxCreateArray(m_Context, VX_TYPE_KEYPOINT, MAX_FOUNDED_CORNERS);
    vx_scalar  optf_estimate_s   = vxCreateScalar(m_Context, VX_TYPE_FLOAT32, &optflow_estimate);
    vx_scalar  optf_max_iter_s   = vxCreateScalar(m_Context, VX_TYPE_UINT32, &optflow_max_iter);
    vx_scalar  optf_init_estim   = vxCreateScalar(m_Context, VX_TYPE_BOOL, &optflow_init_estimate);
    vx_pyramid pyramid_1         = vxCreatePyramid(m_Context, pyramid_level, pyramid_scale, width, height, VX_DF_IMAGE_U8);
    vx_pyramid pyramid_2         = vxCreatePyramid(m_Context, pyramid_level, pyramid_scale, width, height, VX_DF_IMAGE_U8);
    /***      Check objects   ***/
    CHECK_NULL(fast_thresh_s);
    CHECK_NULL(fast_num_corn_s);
    CHECK_NULL(fast_found_corn_s);
    CHECK_NULL(optf_moved_corn_s);
    CHECK_NULL(optf_extim_corn_s);
    CHECK_NULL(pyramid_1);
    CHECK_NULL(pyramid_2);
    /***    End of objects    ***/

    CHECK_NULL( vxRGBtoGrayNode(m_OptFlowGraph, (vx_image)vxGetReferenceFromDelay(m_Images, 0), gray_image_1) );
    CHECK_NULL( vxRGBtoGrayNode(m_OptFlowGraph, (vx_image)vxGetReferenceFromDelay(m_Images, 1), gray_image_2) );
    CHECK_NULL( vxFastCornersNode(m_OptFlowGraph, gray_image_1, fast_thresh_s, vx_true_e, fast_found_corn_s, fast_num_corn_s) );

    CHECK_NULL( vxGaussianPyramidNode(m_OptFlowGraph, gray_image_1, pyramid_1) );
    CHECK_NULL( vxGaussianPyramidNode(m_OptFlowGraph, gray_image_2, pyramid_2) );
    CHECK_NULL( vxOpticalFlowPyrLKNode(m_OptFlowGraph, pyramid_1, pyramid_2, fast_found_corn_s,
                                       optf_extim_corn_s, optf_moved_corn_s, optflow_term,
                                       optf_estimate_s, optf_max_iter_s, optf_init_estim, optflow_wnd_size) );

    CHECK_STATUS( vxVerifyGraph(m_OptFlowGraph) );

    /*********** LOG ******************/
    //CHECK_STATUS(vxAccessScalarValue(fast_num_corn_s, &corners_num));
    //VX_PRINT(VX_ZONE_LOG, "%d found corners\n", corners_num);
    /**********************************/
}

vx_status VXVideoStub::CopyImage(vx_image from, vx_image to)
{
    vx_status status = VX_SUCCESS;

    vx_df_image in_format; vx_uint32 in_width, in_height;
    status |= vxQueryImage(from, VX_IMAGE_ATTRIBUTE_FORMAT, &in_format, sizeof(in_format));
    status |= vxQueryImage(from, VX_IMAGE_ATTRIBUTE_WIDTH,  &in_width,  sizeof(in_width));
    status |= vxQueryImage(from, VX_IMAGE_ATTRIBUTE_HEIGHT, &in_height, sizeof(in_height));
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't access to input image!\n");
        return VX_FAILURE;
    }

    vx_df_image out_format; vx_uint32 out_width, out_height;
    status |= vxQueryImage(to, VX_IMAGE_ATTRIBUTE_FORMAT, &out_format, sizeof(out_format));
    status |= vxQueryImage(to, VX_IMAGE_ATTRIBUTE_WIDTH,  &out_width,  sizeof(out_width));
    status |= vxQueryImage(to, VX_IMAGE_ATTRIBUTE_HEIGHT, &out_height, sizeof(out_height));
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't access to output image!\n");
        return VX_FAILURE;
    }

    if( in_format != out_format || in_height != out_height || in_width != out_width)
    {
        VX_PRINT(VX_ZONE_ERROR, "Input and output images differ!\n");
        return VX_FAILURE;
    }

    void* in_buff = NULL, *out_buff = NULL;
    vx_imagepatch_addressing_t in_addr, out_addr;
    vx_rectangle_t rect = {0, 0, in_width, in_height};
    for(int i = 0; i < 3; i ++)
    {
        status |= vxAccessImagePatch(from, &rect, i, &in_addr, (void **)&in_buff, VX_READ_AND_WRITE);
        if(status != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Can't access input image patch!\n");
            return VX_FAILURE;
        }

        status |= vxAccessImagePatch(to, &rect, i, &out_addr, (void **)&out_buff, VX_READ_AND_WRITE);
        if(status != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Can't access output image patch!\n");
            return VX_FAILURE;
        }

        vx_size size = vxComputeImagePatchSize(from, &rect, i);
        memcpy(out_buff, in_buff, size);

        status |= vxCommitImagePatch(from, NULL, i, &in_addr, in_buff);
        if(status != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Can't commit input image patch!\n");
            return VX_FAILURE;
        }
        status |= vxCommitImagePatch(to, NULL, i, &out_addr, out_buff);
        if(status != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Can't commit output image patch!\n");
            return VX_FAILURE;
        }
    }
    return VX_SUCCESS;
}


vx_image VXVideoStub::NewImage()
{
    if(m_Images == NULL)
    {
        VX_PRINT(VX_ZONE_ERROR, "Pipeline wasn't created!\n");
        return NULL;
    }

    if(m_CurImageId < m_NumImages)
    {
        vx_image ret = (vx_image)vxGetReferenceFromDelay(m_Images, m_CurImageId);
        m_CurImageId++;
        return ret;
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't store more image, need calculate!\n");
        return NULL;
    }
}

vx_image VXVideoStub::Calculate()
{
    if(m_CurImageId < m_NumImages)
        return NULL;

    if(vxProcessGraph(m_OptFlowGraph) != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Graph process error!\n");
        return NULL;
    }
    m_CurImageId--;
    return (vx_image)1;
}

