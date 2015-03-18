#ifndef VX_MODULE_H
#define VX_MODULE_H

#include <initializer_list>
#include <vector>
#include "VX/vx.h"
#include "vx_debug.h"
#include "add_kernels/add_kernels.h"

class VXVideoStub
{
public:
    VXVideoStub();
    virtual ~VXVideoStub();

    vx_status CreatePipeline(const vx_uint32 width, const vx_uint32 height, const vx_uint32 gauss_size);
    vx_status EnableDebug(const std::initializer_list<vx_enum>& zones);
    vx_image  NewImage();
    vx_image  Calculate();
private:
    // vx_status CopyImage(vx_image from, vx_image to);

    vx_context m_Context;
    vx_graph   m_OptFlowGraph;
    vx_graph   m_WarpGraph;
    vx_delay   m_Images;
    vx_delay   m_Matrices;
    vx_image   m_OutImage;

    vx_int32   m_NumImages;
    vx_int32   m_CurImageId;
    vx_bool    m_ImageAdded;
};

#endif // VX_MODULE_H
