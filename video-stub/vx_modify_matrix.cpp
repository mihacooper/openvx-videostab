#include "vx_pipelines.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"


static void TransformPerspective(vx_coordinates2d_t point, vx_coordinates2d_t& tpoint, const vx_float32 m[])
{
    vx_float32 z = point.x * m[6] + point.y * m[7] + m[8];
    tpoint.x = (point.x * m[0] + point.y * m[1] + m[2]) / z;
    tpoint.y = (point.x * m[3] + point.y * m[4] + m[5]) / z;
}

static bool CheckPointArea(vx_coordinates2d_t point, vx_rectangle_t rect)
{
    if(point.x < rect.start_x || point.x >= rect.end_x ||
            point.y < rect.start_y || point.y >= rect.end_y)
        return false;
    return true;
}

static bool CheckRectArea(vx_coordinates2d_t points[4], vx_rectangle_t rect, const vx_float32 m[])
{
    bool in_area = true;
    for(int i = 0; i < 4; i++)
    {
        TransformPerspective(points[i], points[i], m);
        in_area &= CheckPointArea(points[i], rect);
    }
    return in_area;
}

#define ACCURACY 0.1

vx_status ModifyMatrix(vx_matrix matr, vx_uint32 width, vx_uint32 height, vx_float32 scale)
{
    vx_rectangle_t rect = { 0, 0, width, height};
    vx_uint32 w_mod = (width * (1 - scale)) / 2.;
    vx_uint32 h_mod = (height * (1 - scale)) / 2.;
    vx_coordinates2d_t srect[4] = { {w_mod, h_mod}, {width - w_mod, h_mod},
                                    {width - w_mod, height - h_mod}, {w_mod, height - h_mod}};
    vx_float32 m[9];
    vx_float32 inv[9];
    vxAccessMatrix(matr, m);

    // invert
    cv::Mat_<float> cv_matr(3, 3), cv_inv(3, 3);
    memcpy(cv_matr.data, m, sizeof(vx_float32) * 9);
    cv_inv = cv_matr.inv();
    memcpy(inv, cv_inv.data,sizeof(vx_float32) * 9);
    // check need modify
    bool do_modify = !CheckRectArea(srect, rect, inv);

    if(do_modify)
    {
        vx_float32 left = 0., right = 1.;
        cv::Mat_<float> cv_eye = cv::Mat::eye(3, 3, CV_32FC1);
        cv::Mat_<float> cv_res(3, 3);
        while(right - left > ACCURACY)
        {
            vx_float32 alpha = (left + right) / 2.;
            cv_res = alpha * cv_matr + (1. - alpha) * cv_eye;
            cv_inv = cv_res.inv();
            bool check = CheckRectArea(srect, rect, (vx_float32*)cv_inv.data);
            if(check) left  = alpha;
            else      right = alpha;
        }
        memcpy(m, cv_res.data, sizeof(vx_float32) * 9);
    }
    vxCommitMatrix(matr, m);
    return VX_SUCCESS;
}

