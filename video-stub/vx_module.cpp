#include "vx_module.h"
#include "math.h"
#include "memory.h"
#include <cstdio>
#include <string>
#include <ctime>

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

    vx_status status;
    vx_int32 gauss_size = params.warp_gauss.gauss_size;
    m_NumImages = gauss_size * 2 + 1;
    m_NumMatr = gauss_size * 2;

    /*** Calc gauss coeffs ***/
    vx_float32 sigma = gauss_size * 0.7;
    vx_float32* matr_coeffs = params.warp_gauss.gauss_coeffs = new vx_float32[m_NumImages];
    for (int i = -gauss_size; i <= gauss_size; ++i)
        matr_coeffs[i + gauss_size] = ( exp(-i * i / (2.f * sigma * sigma)) );
    vx_float32 sum = 0.;
    for (int i = 0; i < m_NumImages; ++i)
        sum += matr_coeffs[i];
    sum = 1. / sum;
    for (int i = 0; i < m_NumImages; ++i)
    {
        matr_coeffs[i] *= sum;
    }
    /*************************/

    vx_image tmp_image = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    m_Images = vxCreateDelay(m_Context, (vx_reference)tmp_image, m_NumImages);
    CHECK_NULL(m_Images);

    vx_matrix tmp_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    m_Matrices = vxCreateDelay(m_Context, (vx_reference)tmp_matr, m_NumMatr);

    status = FindWarpGraph(m_Context, m_OptFlowGraph,
                  (vx_image)vxGetReferenceFromDelay(m_Images, 1),
                  (vx_image)vxGetReferenceFromDelay(m_Images, 0),
                  (vx_matrix)vxGetReferenceFromDelay(m_Matrices, 0),
                  params.find_warp,
                  m_OptFlowNodes);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create FindWarp graph\n");
        return VX_FAILURE;
    }

    vx_matrix matrices[m_NumMatr];
    for(int i = 0; i < m_NumMatr; i++)
        matrices[i] = (vx_matrix)vxGetReferenceFromDelay(m_Matrices, i);

    m_ResMatr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    m_OutImage = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    status = WarpGaussGraph(m_Context, m_WarpGraph,
                (vx_image)vxGetReferenceFromDelay(m_Images, m_NumImages / 2),
                m_OutImage,
                matrices,
                m_ResMatr,
                params.warp_gauss,
                m_WarpNodes);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create WarpGauss graph\n");
        return VX_FAILURE;
    }
    delete[] params.warp_gauss.gauss_coeffs;
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
    else
    {
        //ret = CopyImage((vx_image)vxGetReferenceFromDelay(m_Images, 0));
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

