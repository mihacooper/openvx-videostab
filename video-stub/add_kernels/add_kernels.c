#include "add_kernels.h"

VX_SAMPLE_API vx_node vxRGBtoGrayNode(vx_graph graph, vx_image input, vx_image output)
{
   vx_node node = 0;
   vx_context context = vxGetContext((vx_reference)graph);
   vx_status status = vxLoadKernels(context, VX_ADD_LIBRARY_NAME);
   if (status == VX_SUCCESS)
   {
      vx_reference params[] = {
         (vx_reference)input,
         (vx_reference)output,
      };
      node = vxCreateNodeByStructure(graph,
                                     VX_ADD_KERNEL_RGB_TO_GRAY,
                                     params,
                                     dimof(params));
   }
   return node;
}
