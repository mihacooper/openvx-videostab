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

    m_width = width;
    m_height = height;
    m_params = params;
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
    vx_image warpedImage = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    m_OutImage = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);

    status = MatrixGaussGraph(m_Context, m_MatrGaussGraph, width, height,
                matrices,
                m_ResMatr,
                params.warp_gauss,
                m_WarpNodes);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create MatrixGauss graph\n");
        return VX_FAILURE;
    }

    status = WarpGraph(m_Context, m_WarpGraph,
                (vx_image)vxGetReferenceFromDelay(m_Images, m_NumImages / 2),
                warpedImage,
                m_ResMatr,
                params.warp_gauss,
                m_WarpNodes);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create Warp graph\n");
        return VX_FAILURE;
    }

    status = CutGraph(m_Context, m_CutGraph, warpedImage, m_OutImage, width, height, params.scale);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create Cut graph\n");
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
        if(vxProcessGraph(m_MatrGaussGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "MatrixGauss graph process error!\n");
            return NULL;
        }
        if(ModifyMatrix(m_ResMatr, m_width, m_height, m_params.scale) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Modifying matrix error!\n");
            return NULL;
        }
        if(vxProcessGraph(m_WarpGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Warp graph process error!\n");
            return NULL;
        }
        if(vxProcessGraph(m_CutGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Cut graph process error!\n");
            return NULL;
        }
        ret = CopyImage(m_OutImage);
        //vx_int32 area = DefineValidRect(m_OutImage, m_ResMatr);
        //m_MaxArea = max(m_MaxArea, area);
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

