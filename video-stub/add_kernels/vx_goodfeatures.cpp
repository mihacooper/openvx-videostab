#include <vx_internal.h>
#include <add_kernels.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

static vx_status VX_CALLBACK vxGoodFeaturesKernel(vx_node node, vx_reference parameters[], vx_uint32 num)
{
   if(num != 2)
       return VX_ERROR_INVALID_PARAMETERS;

   vx_image image = (vx_image)parameters[0];
   vx_array array = (vx_array)parameters[1];

   vx_status status = VX_SUCCESS;
   vx_uint32 width = 0, height = 0;
   void *img_buff = NULL;
   vx_imagepatch_addressing_t addr;
   vx_rectangle_t rect;

   status  = vxGetValidRegionImage(image, &rect);
   status |= vxAccessImagePatch(image, &rect, 0, &addr, (void **)&img_buff, VX_READ_AND_WRITE);
   height = addr.dim_y; width = addr.dim_x;
   if(status != VX_SUCCESS)
   {
      VX_PRINT(VX_ZONE_ERROR, "Can not access to image!\n");
      return status;
   }

   /*
   vx_uint8* cv_buff = malloc(width * height);
   for(int i = 0; i < height; i++)
   {
      for(int j = 0; j < width; j++)
      {
         cv_buff[i * height + j] = img_buff[i * height + j];
      }
   }
   */
   cv::Mat cvImage(cv::Size(width, height), CV_8UC1, img_buff, cv::Mat::AUTO_STEP);

   std::vector<cv::Point2f> points;
   cv::goodFeaturesToTrack(cvImage, points, 1000, 0.01, 10);

   //for(int i = 0; i < points.size(); i++)
   //   cvImage.at<unsigned char>((int)points[i].y, (int)points[i].x) = 0;
   //cv::imshow("Video Stabilization", cvImage);
   //cv::waitKey(5);


   vx_size capacity;
   status |= vxQueryArray(array, VX_ARRAY_ATTRIBUTE_CAPACITY, &capacity, sizeof(capacity));
   vx_size range = capacity > points.size() ? points.size() : capacity;
   void* buff = malloc(range * sizeof(vx_keypoint_t));
   for(int i = 0; i < range; i++)
   {
      vx_keypoint_t* elem = (vx_keypoint_t*)buff + i;
      elem->x = points[i].x;
      elem->y = points[i].y;
   }
   status |= vxTruncateArray(array, 0);
   status |= vxAddArrayItems(array, range, buff, 0);
   free(buff);

   status |= vxCommitImagePatch(image, NULL, 0, &addr, img_buff);
   return status;
}

static vx_status VX_CALLBACK vxGoodFeaturesInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_image input = 0;
            status = vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(input));
            if ((status == VX_SUCCESS) && (input))
            {
                vx_df_image format = 0;
                status = vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
                if ((status == VX_SUCCESS) && (format == VX_DF_IMAGE_U8))
                {
                    status = VX_SUCCESS;
                }
                vxReleaseImage(&input);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxGoodFeaturesOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 1)
    {
        ptr->type = VX_TYPE_ARRAY;
        ptr->dim.array.item_type = VX_TYPE_KEYPOINT;
        ptr->dim.array.capacity = 0; /* no defined capacity requirement */
        status = VX_SUCCESS;
    }
    return status;
}

static vx_param_description_t good_features_kernel_params[] = {
    {VX_INPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
    {VX_OUTPUT, VX_TYPE_ARRAY, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t good_features_kernel = {
    VX_ADD_KERNEL_GOOD_FEATURES,
    "org.khronos.openvx.good_features",
    vxGoodFeaturesKernel,
    good_features_kernel_params, dimof(good_features_kernel_params),
    vxGoodFeaturesInputValidator,
    vxGoodFeaturesOutputValidator,
};
