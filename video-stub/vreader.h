#ifndef __VREADER__
#define __VREADER__

#include <string>
#include "VX/vx.h"
#include "opencv2/highgui/highgui.hpp"

class VReader
{
public:
    VReader(std::string filename);
    virtual ~VReader();

    bool ReadFrame(vx_context context, vx_image& frame);
    void ShowFrame(vx_image& frame);

private:
    cv::VideoCapture m_cvReader;
};

#endif // __VREADER__
