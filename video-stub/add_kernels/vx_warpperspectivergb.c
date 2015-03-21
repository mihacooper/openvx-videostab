#include "add_kernels.h"
#include "vx_internal.h"

static vx_bool read_pixel(void *base, vx_imagepatch_addressing_t *addr,
                          vx_float32 x, vx_float32 y, const vx_border_mode_t *borders, vx_uint32 *pixel)
{
    vx_bool out_of_bounds = (x < 0 || y < 0 || x >= addr->dim_x || y >= addr->dim_y);
    vx_uint32 bx, by;
    vx_uint32 *bpixel;
    if (out_of_bounds)
    {
        if (borders->mode == VX_BORDER_MODE_UNDEFINED)
            return vx_false_e;
        if (borders->mode == VX_BORDER_MODE_CONSTANT)
        {
            *pixel = borders->constant_value;
            return vx_true_e;
        }
    }

    // bounded x/y
    bx = x < 0 ? 0 : x >= addr->dim_x ? addr->dim_x - 1 : (vx_uint32)x;
    by = y < 0 ? 0 : y >= addr->dim_y ? addr->dim_y - 1 : (vx_uint32)y;

    bpixel = vxFormatImagePatchAddress2d(base, bx, by, addr);
    *pixel = *bpixel;

    return vx_true_e;
}

static void transform_perspective(vx_uint32 dst_x, vx_uint32 dst_y, const vx_float32 m[], vx_float32 *src_x, vx_float32 *src_y)
{
    vx_float32 z = dst_x * m[6] + dst_y * m[7] + m[8];

    *src_x = (dst_x * m[0] + dst_y * m[1] + m[2]) / z;
    *src_y = (dst_x * m[3] + dst_y * m[4] + m[5]) / z;
}

static vx_status VX_CALLBACK vxWarpPerspectiveRGBKernel(vx_node node, vx_reference *parameters, vx_uint32 num)
{
    vx_image  src_image = (vx_image) parameters[0];
    vx_matrix matrix    = (vx_matrix)parameters[1];
    vx_scalar stype     = (vx_scalar)parameters[2];
    vx_image  dst_image = (vx_image) parameters[3];

    vx_border_mode_t borders;
    vxQueryNode(node, VX_NODE_ATTRIBUTE_BORDER_MODE, &borders, sizeof(borders));

    vx_status status = VX_SUCCESS;
    void *src_base = NULL;
    void *dst_base = NULL;
    vx_imagepatch_addressing_t src_addr, dst_addr;
    vx_uint32 dst_width, dst_height;
    vx_rectangle_t src_rect;
    vx_rectangle_t dst_rect;

    vx_float32 m[9];
    vx_enum type = 0;

    vx_uint32 y = 0u, x = 0u;

    vxQueryImage(dst_image, VX_IMAGE_ATTRIBUTE_WIDTH, &dst_width, sizeof(dst_width));
    vxQueryImage(dst_image, VX_IMAGE_ATTRIBUTE_HEIGHT, &dst_height, sizeof(dst_height));

    vxGetValidRegionImage(src_image, &src_rect);
    dst_rect.start_x = 0;
    dst_rect.start_y = 0;
    dst_rect.end_x = dst_width;
    dst_rect.end_y = dst_height;

    status |= vxAccessImagePatch(src_image, &src_rect, 0, &src_addr, &src_base, VX_READ_ONLY);
    status |= vxAccessImagePatch(dst_image, &dst_rect, 0, &dst_addr, &dst_base, VX_WRITE_ONLY);

    status |= vxAccessMatrix(matrix, m);
    status |= vxAccessScalarValue(stype, &type);

    printf("/*******************/\n");
    printf("%f,%f,%f\n%f,%f,%f\n%f,%f,%f\n",
        m[0],m[1],m[2],
        m[3],m[4],m[5],
        m[6],m[7],m[8]);
    printf("/*******************/\n");

    if (status == VX_SUCCESS)
    {
        for (y = 0u; y < dst_addr.dim_y; y++)
        {
            for (x = 0u; x < dst_addr.dim_x; x++)
            {
                vx_uint32 *dst = vxFormatImagePatchAddress2d(dst_base, x, y, &dst_addr);

                vx_float32 xf;
                vx_float32 yf;
                transform_perspective(x, y, m, &xf, &yf);
                xf -= (vx_float32)src_rect.start_x;
                yf -= (vx_float32)src_rect.start_y;

                if (type == VX_INTERPOLATION_TYPE_NEAREST_NEIGHBOR)
                {
                    read_pixel(src_base, &src_addr, xf, yf, &borders, dst);
                }
                else if (type == VX_INTERPOLATION_TYPE_BILINEAR)
                {
                    vx_uint32 tl = 0, tr = 0, bl = 0, br = 0;
                    vx_bool defined = vx_true_e;
                    defined &= read_pixel(src_base, &src_addr, floorf(xf), floorf(yf), &borders, &tl);
                    defined &= read_pixel(src_base, &src_addr, floorf(xf) + 1, floorf(yf), &borders, &tr);
                    defined &= read_pixel(src_base, &src_addr, floorf(xf), floorf(yf) + 1, &borders, &bl);
                    defined &= read_pixel(src_base, &src_addr, floorf(xf) + 1, floorf(yf) + 1, &borders, &br);
                    if (defined)
                    {
                        vx_float32 ar = xf - floorf(xf);
                        vx_float32 ab = yf - floorf(yf);
                        vx_float32 al = 1.0f - ar;
                        vx_float32 at = 1.0f - ab;
                        *dst = tl * al * at + tr * ar * at + bl * al * ab + br * ar * ab;
                    }
                }
            }
        }
    }

    status |= vxCommitMatrix(matrix, m);
    status |= vxCommitImagePatch(src_image, NULL, 0, &src_addr, src_base);
    status |= vxCommitImagePatch(dst_image, &dst_rect, 0, &dst_addr, dst_base);

    return status;
}


static vx_status VX_CALLBACK vxWarpPerspectiveRGBInputValidator(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 0)
    {
        vx_image input = 0;
        vx_parameter param = vxGetParameterByIndex(node, index);

        vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &input, sizeof(input));
        if (input)
        {
            vx_df_image format = 0;
            vxQueryImage(input, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
            if (format == VX_DF_IMAGE_RGB)
            {
                status = VX_SUCCESS;
            }
            vxReleaseImage(&input);
        }
        vxReleaseParameter(&param);
    }
    else if (index == 1)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_matrix matrix;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &matrix, sizeof(matrix));
            if (matrix)
            {
                vx_enum data_type = 0;
                vx_size rows = 0ul, columns = 0ul;
                vxQueryMatrix(matrix, VX_MATRIX_ATTRIBUTE_TYPE, &data_type, sizeof(data_type));
                vxQueryMatrix(matrix, VX_MATRIX_ATTRIBUTE_ROWS, &rows, sizeof(rows));
                vxQueryMatrix(matrix, VX_MATRIX_ATTRIBUTE_COLUMNS, &columns, sizeof(columns));
                if ((data_type == VX_TYPE_FLOAT32) && (columns == 3) && (rows == 3))
                {
                    status = VX_SUCCESS;
                }
                vxReleaseMatrix(&matrix);
            }
            vxReleaseParameter(&param);
        }
    }
    else if (index == 2)
    {
        vx_parameter param = vxGetParameterByIndex(node, index);
        if (param)
        {
            vx_scalar scalar = 0;
            vxQueryParameter(param, VX_PARAMETER_ATTRIBUTE_REF, &scalar, sizeof(scalar));
            if (scalar)
            {
                vx_enum stype = 0;
                vxQueryScalar(scalar, VX_SCALAR_ATTRIBUTE_TYPE, &stype, sizeof(stype));
                if (stype == VX_TYPE_ENUM)
                {
                    vx_enum interp = 0;
                    vxAccessScalarValue(scalar, &interp);
                    if ((interp == VX_INTERPOLATION_TYPE_NEAREST_NEIGHBOR) ||
                        (interp == VX_INTERPOLATION_TYPE_BILINEAR))
                    {
                        status = VX_SUCCESS;
                    }
                    else
                    {
                        status = VX_ERROR_INVALID_VALUE;
                    }
                }
                else
                {
                    status = VX_ERROR_INVALID_TYPE;
                }
                vxReleaseScalar(&scalar);
            }
            vxReleaseParameter(&param);
        }
    }
    return status;
}

static vx_status VX_CALLBACK vxWarpPerspectiveRGBOutputValidator(vx_node node, vx_uint32 index, vx_meta_format_t *ptr)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;
    if (index == 3)
    {
        vx_parameter dst_param = vxGetParameterByIndex(node, index);
        if (dst_param)
        {
            vx_image dst = 0;
            vxQueryParameter(dst_param, VX_PARAMETER_ATTRIBUTE_REF, &dst, sizeof(dst));
            if (dst)
            {
                vx_uint32 w1 = 0, h1 = 0;
                vx_df_image f1 = VX_DF_IMAGE_VIRT;

                vxQueryImage(dst, VX_IMAGE_ATTRIBUTE_WIDTH, &w1, sizeof(w1));
                vxQueryImage(dst, VX_IMAGE_ATTRIBUTE_HEIGHT, &h1, sizeof(h1));
                vxQueryImage(dst, VX_IMAGE_ATTRIBUTE_FORMAT, &f1, sizeof(f1));
                /* output can not be virtual */
                if ((w1 != 0) && (h1 != 0) && (f1 == VX_DF_IMAGE_RGB))
                {
                    /* fill in the meta data with the attributes so that the checker will pass */
                    ptr->type = VX_TYPE_IMAGE;
                    ptr->dim.image.format = VX_DF_IMAGE_RGB;
                    ptr->dim.image.width = w1;
                    ptr->dim.image.height = h1;
                    status = VX_SUCCESS;
                }
                vxReleaseImage(&dst);
            }
            vxReleaseParameter(&dst_param);
        }
    }
    return status;
}

static vx_param_description_t add_warp_perspective_rgb_kernel_params[] = {
    {VX_INPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT, VX_TYPE_MATRIX, VX_PARAMETER_STATE_REQUIRED},
    {VX_INPUT, VX_TYPE_SCALAR, VX_PARAMETER_STATE_OPTIONAL},
    {VX_OUTPUT, VX_TYPE_IMAGE, VX_PARAMETER_STATE_REQUIRED},
};

vx_kernel_description_t add_warp_perspective_rgb_kernel = {
    VX_ADD_KERNEL_WARP_PERSPECTIVE_RGB,
    VX_ADD_KERNEL_NAME_WARP_PERSPECTIVE_RGB,
    vxWarpPerspectiveRGBKernel,
    add_warp_perspective_rgb_kernel_params, dimof(add_warp_perspective_rgb_kernel_params),
    vxWarpPerspectiveRGBInputValidator,
    vxWarpPerspectiveRGBOutputValidator,
    NULL, NULL
};

