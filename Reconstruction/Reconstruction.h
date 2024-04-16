//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_RECONSTRUCTION_H
#define ARDUCV_RECONSTRUCTION_H

#include <boost/json.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <utility>
#include "../Types/Types.h"
#include "../Tools/Tools.h"
#include "../Param/CameraParam.h"
#include "../Param/Settings.h"

namespace ArduCV {

    extern cv::Mat disp, disp8;
    extern cv::Mat _3dImage;

    class Reconstruction {
    public:
        explicit Reconstruction(CameraParamStereo _param, const int &_cameraDeviceNo = 0);

        ~Reconstruction();

        void InteractiveReconstruction();

        std::pair<cv::Mat, cv::Mat> StereoMatchBM(const cv::UMat& left, const cv::UMat& right);

        std::pair<cv::Mat, cv::Mat> StereoMatchBMPartitionalParallel(cv::UMat left, cv::UMat right);
    private:
        CameraParamStereo param;
        int cameraDeviceNo;
        int numDisparities = 16;
        int blockSize = 6;
        int uniquenessRatio = 3;
        int preFilterCap = 16;
        bool parallel = false;
        cv::Rect ROI1, ROI2;
        cv::Mat mapLx, mapLy, mapRx, mapRy, Q;
        bool rectify = false;
        cv::VideoCapture capture;

        static void onClick(int event, int x, int y, int z, void *userdata) {
            if (event == cv::EVENT_LBUTTONDOWN) {
                CameraParamStereo *param = static_cast<CameraParamStereo *>(userdata);
                auto coordinate = _3dImage.at<cv::Vec3f>(cv::Point(x, y));
                std::cout << HIGHLIGHT "3D point is :" << coordinate << "\n" CLRST;
                auto dispVal = (disp.at<uint16_t>(y, x)) / 16;
                auto baseline = cv::norm(param->translationMatrix) / 1000;
                auto focal_length = param->cameraMatrixLeft.at<double>(0, 0);
                auto depth = (baseline * focal_length) / (dispVal);
                depth = depth * 100;
                Console::WriteLine("Real distance is: "+std::to_string(depth));
            }
        }

        void StereoRectify(const cv::Size &imageSize) {
            cv::Mat R1, R2, P1, P2;
            // cv::stereoRectify(alpha: -1:Auto, 0:No black area, 1:Full picture )
            if (!parallel) {
                cv::stereoRectify(param.cameraMatrixLeft, param.distCoeffsLeft, param.cameraMatrixRight,
                                  param.distCoeffsRight,
                                  imageSize, param.rotationMatrix, param.translationMatrix, R1, R2, P1, P2, Q,
                                  cv::CALIB_ZERO_DISPARITY,
                                  1, imageSize, &ROI1, &ROI2);
            } else {
                cv::stereoRectify(param.cameraMatrixLeft, param.distCoeffsLeft, param.cameraMatrixRight,
                                  param.distCoeffsRight,
                                  imageSize, param.rotationMatrix, param.translationMatrix, R1, R2, P1, P2, Q,
                                  cv::CALIB_ZERO_DISPARITY,
                                  0, imageSize, &ROI1, &ROI2);
            }
            cv::initUndistortRectifyMap(param.cameraMatrixLeft, param.distCoeffsLeft, R1, P1, imageSize, CV_16SC2,
                                        mapLx, mapLy);
            cv::initUndistortRectifyMap(param.cameraMatrixRight, param.distCoeffsRight, R2, P2, imageSize, CV_16SC2,
                                        mapRx, mapRy);
        }
    };
}


#endif //ARDUCV_RECONSTRUCTION_H
