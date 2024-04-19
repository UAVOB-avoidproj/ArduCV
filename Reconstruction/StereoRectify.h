//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_STEREORECTIFY_H
#define ARDUCV_STEREORECTIFY_H

#include <opencv2/opencv.hpp>
#include "../param/CameraParam.h"
#include "../Types/Types.h"

namespace ArduCV {
    class StereoRectify {
    public:
        cv::Mat R1, R2, P1, P2, Q;
        Roi ROI1, ROI2;
        Map mapLx, mapRx, mapLy, mapRy;

        std::tuple<Roi, Roi, Map, Map, Map, Map, cv::Mat>
        RectifyCalculate(const CameraParamStereo &_param, const cv::Size &imageSize);

        std::pair<cv::Mat, cv::Mat> Remap(const cv::Mat &Left, const cv::Mat &Right);
        std::pair<cv::UMat, cv::UMat> Remap(const cv::UMat &Left, const cv::UMat &Right);
    };
};


#endif //ARDUCV_STEREORECTIFY_H
