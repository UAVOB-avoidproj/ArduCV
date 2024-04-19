//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_DISTANCEDETECTION_H
#define ARDUCV_DISTANCEDETECTION_H

#include <boost/json.hpp>
#include <opencv2/opencv.hpp>
#include <tbb/tbb.h>
#include <iostream>
#include <string>
#include <utility>
#include <chrono>
#include <numeric>
#include "StereoRectify.h"
#include "StereoBM.h"
#include "../Types/Types.h"
#include "../Tools/Tools.h"
#include "../param/CameraParam.h"
#include "../Param/Settings.h"

namespace ArduCV {
    /**
     * @brief Detect distance between barrier
     */
    class DistanceDetection {
    public:
        explicit DistanceDetection(CameraParamStereo _param, const int &_cameraDeviceNo = 0);

        ~DistanceDetection();

        void startContinuousDetection(const std::function<void(DistanceDataFrame)> &subscriber,int nearThres);

        void cancelTask();

    private:
        int cameraDeviceNo = 0;
        int numDisparities = 16;
        int blockSize = 6;
        int uniquenessRatio = 3;
        int preFilterCap = 16;
        std::atomic<bool>cancellationToken = false;
        cv::Mat disp, disp8, _3dImage;
        CameraParamStereo camParam;

        static std::chrono::duration<long long, std::ratio<1, 1000000>> getEpoch();
    };



} // ArduCV

#endif //ARDUCV_DISTANCEDETECTION_H
