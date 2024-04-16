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

        static void onClick(int event, int x, int y, int z, void *userdata);

        void StereoRectify(const cv::Size &imageSize);
    };
}


#endif //ARDUCV_RECONSTRUCTION_H
