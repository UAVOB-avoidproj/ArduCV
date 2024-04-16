//
// Created by Kira on 24-4-16.
//

#include <opencv2/opencv.hpp>
#include "../Types/Types.h"
#include "../Tools/Tools.h"

#ifndef ARDUCV_CAMERAPARAM_H
#define ARDUCV_CAMERAPARAM_H


namespace ArduCV {
    struct CamaraParam {
        bool checkRange = false;
        CameraMatrix cameraMatrix;
        DistCoeffs distCoeffs;
        Rvecs rotationVectors;
        Tvecs translationVectors;
        double rms = 0;
    };

    struct CameraParamStereo {
        CamaraParam left;
        CamaraParam right;
        double rms = 0;
        CameraMatrix cameraMatrixLeft;
        DistCoeffs distCoeffsLeft;
        CameraMatrix cameraMatrixRight;
        DistCoeffs distCoeffsRight;
        Rmat rotationMatrix;
        Tmat translationMatrix;
        Emat essentialMatrix;
        Fmat fundamentalMatrix;
    };

    extern CameraParamStereo param;

    bool saveCalibrateData(const CameraParamStereo &data, const std::string &path = "calibration_data.yaml");

    std::pair<bool, CameraParamStereo> readCalibrateData(const std::string &path = "calibration_data.yaml");

    void showCalibrateData(const CameraParamStereo& data);
}
#endif //ARDUCV_CAMERAPARAM_H
