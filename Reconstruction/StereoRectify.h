//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_STEREORECTIFY_H
#define ARDUCV_STEREORECTIFY_H

#include <opencv2/opencv.hpp>
#include "../Param/CameraParam.h"

namespace ArduCV {
    auto StereoRectify(const cv::Size &imageSize) {
        cv::Mat R1, R2, P1, P2;
        // cv::stereoRectify(alpha: -1:Auto, 0:No black area, 1:Full picture )
        cv::stereoRectify(param.cameraMatrixLeft, param.distCoeffsLeft, param.cameraMatrixRight,
                          param.distCoeffsRight,
                          imageSize, param.rotationMatrix, param.translationMatrix, R1, R2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY,
                          1, imageSize, &ROI1, &ROI2);

        cv::initUndistortRectifyMap(param.cameraMatrixLeft, param.distCoeffsLeft, R1, P1, imageSize, CV_16SC2,
                                    mapLx, mapLy);
        cv::initUndistortRectifyMap(param.cameraMatrixRight, param.distCoeffsRight, R2, P2, imageSize, CV_16SC2,
                                    mapRx, mapRy);
        return std::make_tuple<>()
    }
};


#endif //ARDUCV_STEREORECTIFY_H
