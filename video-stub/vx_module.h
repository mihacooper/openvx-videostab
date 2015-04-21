#ifndef VX_MODULE_H
#define VX_MODULE_H

#include <initializer_list>
#include <vector>
#include <map>
#include <string>
#include "VX/vx.h"
#include "vx_debug.h"
#include "add_kernels/add_kernels.h"

struct  VideoStabParams
{
    vx_size    gauss_size;

    /*    FAST9    */
    vx_float32 fast_thresh;
    vx_uint32  fast_max_corners;
    /***************/

    /* GaussianPyramid */
    vx_float32 pyramid_scale;
    vx_size    pyramid_level;
    /*******************/

    /* GaussianPyramid */
    vx_size    optflow_wnd_size;
    vx_enum    optflow_term;
    vx_float32 optflow_estimate;
    vx_uint32  optflow_max_iter;
    /*******************/
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
private:
    vx_image  CopyImage(vx_image input);
    vx_int32  DefineValidRect(vx_image image, vx_matrix matrix);
    void TransformPerspective(vx_uint32& x, vx_uint32& y,  vx_int32& tx, vx_int32& ty, const vx_float32 m[]);

    vx_context m_Context;
    vx_graph   m_OptFlowGraph;
    vx_graph   m_WarpGraph;
    vx_delay   m_Images;
    vx_delay   m_Matrices;
    vx_image   m_OutImage;
    vx_matrix  m_ResMatr;

    vx_int32   m_NumImages;
    vx_int32   m_NumMatr;
    vx_int32   m_CurImageId;
    vx_bool    m_ImageAdded;
    vx_int32   m_MaxArea;
    std::map<std::string, vx_node> m_OptFlowNodes;
    std::map<std::string, vx_node> m_WarpNodes;
};

#endif // VX_MODULE_H
