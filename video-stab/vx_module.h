#ifndef VX_MODULE_H
#define VX_MODULE_H

#include "vx_common.h"
#include "vx_pipelines.h"

struct  VideoStabParams
{
    FindWarpParams  find_warp;
    WarpGaussParams warp_gauss;
};

class VXVideoStab
{
public:
    VXVideoStab();
    virtual ~VXVideoStab();

    vx_status CreatePipeline(const vx_uint32 width, const vx_uint32 height, VideoStabParams& params);
    vx_status EnableDebug(const std::initializer_list<vx_enum>& zones);
    vx_status DisableDebug(const std::initializer_list<vx_enum>& zones);
    vx_image  NewImage();
    vx_image  Calculate();
private:
    /* Context of execution */
    vx_context m_Context;
    /* Graphs */
    vx_graph   m_FindWarpGraph;
    vx_graph   m_WarpAndCutGraph;
    /* Containers */
    vx_delay   m_Images;
    vx_delay   m_Matrices;
    /* One step result image */
    vx_image   m_ResultImage;
    /* Internal status */
    vx_int32   m_CurrState;
    vx_bool    m_ImageAdded;
    /* Global configs */
    vx_int32   m_WorkSize;
    VideoStabParams m_params;
    vx_uint32 m_width, m_height;
};

#endif // VX_MODULE_H
