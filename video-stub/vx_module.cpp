#include "vx_module.h"
#include "math.h"
#include "memory.h"
#include <cstdio>
#include <string>
#include <ctime>

VXVideoStab::VXVideoStab() :
    m_CurrState(0), m_WorkSize(0), m_Images(NULL),
    m_Matrices(NULL), m_FindWarpGraph(NULL), m_WarpAndCutGraph(NULL),
    m_ImageAdded(vx_false_e)
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

vx_status VXVideoStab::DisableDebug(const std::initializer_list<vx_enum>& zones)
{
    for(auto i = zones.begin(); i != zones.end(); i++) \
        vx_clr_debug_zone(*i);
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
    m_WorkSize = gauss_size * 2 + 1;
    int numMatr = gauss_size * 2;

    /*** Calc gauss coeffs ***/
    vx_float32 sigma = gauss_size * 0.7;
    vx_float32* matr_coeffs = params.warp_gauss.gauss_coeffs = new vx_float32[m_WorkSize];
    for (int i = -gauss_size; i <= gauss_size; ++i)
        matr_coeffs[i + gauss_size] = ( exp(-i * i / (2.f * sigma * sigma)) );
    vx_float32 sum = 0.;
    for (int i = 0; i < m_WorkSize; ++i)
        sum += matr_coeffs[i];
    sum = 1. / sum;
    for (int i = 0; i < m_WorkSize; ++i)
    {
        matr_coeffs[i] *= sum;
    }
    /*************************/

    vx_image tmp_image = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    m_Images = vxCreateDelay(m_Context, (vx_reference)tmp_image, m_WorkSize);
    CHECK_NULL(m_Images);

    vx_matrix tmp_matr = vxCreateMatrix(m_Context, VX_TYPE_FLOAT32, 3, 3);
    m_Matrices = vxCreateDelay(m_Context, (vx_reference)tmp_matr, numMatr);

    status = FindWarpGraph(m_Context, m_FindWarpGraph,
                  (vx_image)vxGetReferenceFromDelay(m_Images, 1),
                  (vx_image)vxGetReferenceFromDelay(m_Images, 0),
                  (vx_matrix)vxGetReferenceFromDelay(m_Matrices, 0),
                  params.find_warp);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create FindWarp graph\n");
        return VX_FAILURE;
    }

    vx_matrix matrices[numMatr];
    for(int i = 0; i < numMatr; i++)
        matrices[i] = (vx_matrix)vxGetReferenceFromDelay(m_Matrices, i);

    m_ResultImage = vxCreateImage(m_Context, width, height, VX_DF_IMAGE_RGB);
    status = WarpGaussAndCutGraph(m_Context, m_WarpAndCutGraph,
                (vx_image)vxGetReferenceFromDelay(m_Images, m_WorkSize / 2),
                m_ResultImage,
                matrices,
                params.warp_gauss);
    if(status != VX_SUCCESS)
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't create MatrixGauss graph\n");
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

    if(m_CurrState < m_WorkSize)
    {
        m_ImageAdded = vx_true_e;
        vx_image ret = (vx_image)vxGetReferenceFromDelay(m_Images, 0);
        m_CurrState++;
        return ret;
    }
    else
    {
        VX_PRINT(VX_ZONE_ERROR, "Can't store more images, need calculate!\n");
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
    if(m_CurrState > 1)
    {
        if(vxProcessGraph(m_FindWarpGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "Optical flow graph process error!\n");
            return NULL;
        }
    }
    vx_image ret = NULL;
    if(m_CurrState == m_WorkSize)
    {
        if(vxProcessGraph(m_WarpAndCutGraph) != VX_SUCCESS)
        {
            VX_PRINT(VX_ZONE_ERROR, "MatrixGauss graph process error!\n");
            return NULL;
        }
        ret = m_ResultImage;
        m_CurrState--;
    }
    vxAgeDelay(m_Images);
    vxAgeDelay(m_Matrices);
    m_ImageAdded = vx_false_e;
    return ret;
}

