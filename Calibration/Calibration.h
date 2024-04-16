//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_CALIBRATION_H
#define ARDUCV_CALIBRATION_H

#include <opencv2/opencv.hpp>
#include "../Types/Types.h"
#include "../Tools/Tools.h"
#include "../Param/CameraParam.h"
#include "../Param/Settings.h"

namespace ArduCV {

    class SingleCalibration {
    public:
        SingleCalibration(int _chessBoardHeight, int _chessBoardWidth, float _squareSize);

        std::tuple<bool, Corners> findCorners(const cv::Mat &frame);

        cv::Mat appendCorners(cv::Mat frame, Corners corners);

        std::tuple<bool, CameraMatrix, DistCoeffs, Rvecs, Tvecs, Rms, ImagePoints, ObjectPoints> calculate();

    private:
        float squareSize;
        cv::Size boardSize;
        cv::Size imageSize;
        ImagePoints imagePoints;
        ObjectPoints objectPoints;
    };

    class InteractiveCameraCalibration {
    public:
        InteractiveCameraCalibration(int _chessBoardHeight, int _chessBoardWidth,
                                     float _squareSize, int _cameraDeviceNo = 0, bool _useBinaryThreshold = false);
        ~InteractiveCameraCalibration() ;

        bool startCalibration(CameraParamStereo &cameraParam);

    private:
        cv::Size boardSize;
        float squareSize;
        ObjectPoints objectPoints;
        CameraParamStereo lastCalibrate;
        int cameraDeviceNo;
        bool useBinaryThreshold;
        cv::VideoCapture capture;
    };

} // ArduCV

#endif //ARDUCV_CALIBRATION_H
