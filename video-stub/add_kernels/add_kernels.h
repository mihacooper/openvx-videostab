#ifndef ADD_KERNELS_H
#define ADD_KERNELS_H

#include <VX/vx.h>
#include <VX/vx_api.h>
#include <VX/vx_helper.h>

#define VX_ADD_LIBRARY (0x1)

#define VX_ADD_LIBRARY_NAME "vx_add_kernels"

#ifndef VX_SAMPLE_API
#ifdef WIN32
#ifdef VX_ADD_KERNELS_LIBRARY
#define VX_SAMPLE_API __declspec(dllexport)
#else
#define VX_SAMPLE_API __declspec(dllimport)
#endif
#else
#define VX_SAMPLE_API
#endif
#endif

#ifndef dimof
#define dimof(x)  (sizeof(x)/sizeof(x[0]))
#endif

#define VX_ADD_KERNEL_NAME_RGB_TO_GRAY     "org.openvx.add.rgb_to_gray"

enum vx_kernel_iap_ext_e {
    VX_ADD_KERNEL_RGB_TO_GRAY     = VX_KERNEL_BASE(VX_ID_INTEL, VX_ADD_LIBRARY) + 0x0,
};


#ifdef __cplusplus
extern "C" {
#endif

VX_SAMPLE_API vx_node vxRGBtoGrayNode(vx_graph graph, vx_image input, vx_image output);

#ifdef __cplusplus
}
#endif

#endif // ADD_KERNELS_H
