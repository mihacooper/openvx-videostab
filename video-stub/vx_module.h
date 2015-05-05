#ifndef VX_MODULE_H
#define VX_MODULE_H

#include "vx_common.h"
#include "vx_pipelines.h"

struct  VideoStabParams
{
    vx_float32      scale;
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
    vx_image  NewImage();
    vx_image  Calculate();
    void      PrintPerf();
    double    AvgPerf(vx_perf_t& perf);
    void      NodesPerf(std::map<std::string, vx_node>& nodes);
    vx_image  CutImage(vx_image input);
    vx_status EnableCuting(vx_uint32 width, vx_uint32 height);
private:
    vx_image  CopyImage(vx_image input);
    vx_int32  DefineValidRect(vx_image image, vx_matrix matrix);
    void TransformPerspective(vx_uint32& x, vx_uint32& y,  vx_int32& tx, vx_int32& ty, const vx_float32 m[]);

    vx_context m_Context;
    vx_graph   m_OptFlowGraph;
    vx_graph   m_MatrGaussGraph;
    vx_graph   m_WarpGraph;
    vx_graph   m_CutGraph;
    vx_delay   m_Images;
    vx_delay   m_Matrices;
    vx_image   m_OutImage;
    vx_matrix  m_ResMatr;

    vx_int32   m_NumImages;
    vx_int32   m_NumMatr;
    vx_int32   m_CurImageId;
    vx_bool    m_ImageAdded;
    vx_int32   m_MaxArea;

    VideoStabParams m_params;
    vx_uint32 m_width, m_height;
    std::map<std::string, vx_node> m_OptFlowNodes;
    std::map<std::string, vx_node> m_WarpNodes;
};

#endif // VX_MODULE_H
