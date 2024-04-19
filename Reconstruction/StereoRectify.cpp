//
// Created by Kira on 24-4-16.
//

#include "StereoRectify.h"

namespace ArduCV {
    std::tuple<Roi, Roi, Map, Map, Map, Map, cv::Mat>
    StereoRectify::RectifyCalculate(const CameraParamStereo &_param, const cv::Size &imageSize) {

        // cv::stereoRectify(alpha: -1:Auto, 0:No black area, 1:Full picture )
        cv::stereoRectify(_param.cameraMatrixLeft, _param.distCoeffsLeft, _param.cameraMatrixRight,
                          _param.distCoeffsRight,
                          imageSize, _param.rotationMatrix, _param.translationMatrix, R1, R2, P1, P2, Q,
                          cv::CALIB_ZERO_DISPARITY,
                          1, imageSize, &ROI1, &ROI2);

        cv::initUndistortRectifyMap(_param.cameraMatrixLeft, _param.distCoeffsLeft, R1, P1, imageSize, CV_16SC2,
                                    mapLx, mapLy);
        cv::initUndistortRectifyMap(_param.cameraMatrixRight, _param.distCoeffsRight, R2, P2, imageSize, CV_16SC2,
                                    mapRx, mapRy);
        return std::make_tuple(ROI1, ROI2, mapLx, mapLy, mapRx, mapRy, Q);
    }

    std::pair<cv::Mat, cv::Mat> StereoRectify::Remap(const cv::Mat &Left, const cv::Mat &Right) {
        cv::Mat LeftOut, RightOut;
        cv::remap(Left, LeftOut, mapLx, mapLy, cv::INTER_LINEAR);
        cv::remap(Right, RightOut, mapRx, mapRy, cv::INTER_LINEAR);
        return std::make_pair(LeftOut, RightOut);
    }

    std::pair<cv::UMat, cv::UMat> StereoRectify::Remap(const cv::UMat &Left, const cv::UMat &Right) {
        cv::UMat LeftOut, RightOut;
        cv::remap(Left, LeftOut, mapLx, mapLy, cv::INTER_LINEAR);
        cv::remap(Right, RightOut, mapRx, mapRy, cv::INTER_LINEAR);
        return std::make_pair(LeftOut, RightOut);
    }
}