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

#define VX_ADD_KERNEL_NAME_RGB_TO_GRAY          "org.openvx.add.rgb_to_gray"
#define VX_ADD_KERNEL_NAME_FIND_WARP            "org.openvx.add.find_warp"
#define VX_ADD_KERNEL_NAME_WARP_PERSPECTIVE_RGB "org.openvx.add.warp_perspective_rgb"
#define VX_ADD_KERNEL_NAME_MATRIX_MULTIPLY      "org.openvx.add.matrix_multiply"

enum vx_add_kernel_e {
    VX_ADD_KERNEL_RGB_TO_GRAY          = VX_KERNEL_BASE(VX_ID_INTEL, VX_ADD_LIBRARY) + 0x0,
    VX_ADD_KERNEL_FIND_WARP            = VX_KERNEL_BASE(VX_ID_INTEL, VX_ADD_LIBRARY) + 0x1,
    VX_ADD_KERNEL_WARP_PERSPECTIVE_RGB = VX_KERNEL_BASE(VX_ID_INTEL, VX_ADD_LIBRARY) + 0x2,
    VX_ADD_KERNEL_MATRIX_MULTIPLY      = VX_KERNEL_BASE(VX_ID_INTEL, VX_ADD_LIBRARY) + 0x3,
};


#ifdef __cplusplus
extern "C" {
#endif

VX_SAMPLE_API vx_node vxRGBtoGrayNode(vx_graph graph, vx_image input, vx_image output);
VX_SAMPLE_API vx_node vxFindWarpNode(vx_graph graph, vx_array def_pnts, vx_array moved_pnts, vx_matrix matr);
VX_SAMPLE_API vx_node vxWarpPerspectiveRGBNode(vx_graph graph, vx_image input, vx_matrix matr, vx_scalar inter, vx_image output);
VX_SAMPLE_API vx_node vxMatrixMultiplyNode(vx_graph graph, vx_matrix input1, vx_matrix input2, vx_scalar coeff, vx_matrix output);

#ifdef __cplusplus
}
#endif

#endif // ADD_KERNELS_H
