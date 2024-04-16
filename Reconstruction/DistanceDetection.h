//
// Created by Kira on 24-4-16.
//

#ifndef ARDUCV_DISTANCEDETECTION_H
#define ARDUCV_DISTANCEDETECTION_H

#include <boost/json.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <utility>
#include <chrono>
#include "../Types/Types.h"
#include "../Tools/Tools.h"
#include "../Param/CameraParam.h"
#include "../Param/Settings.h"

namespace ArduCV {
    /**
     * @brief The data frame that subscriber will receive
     */
    struct DistanceDataFrame {
        std::chrono::duration<long long, std::ratio<1, 1000000>> epoch;

    };

    /**
     * @brief Detect distance between barrier
     * @tparam Type The type that subscriber returns
     */
    template<typename Type>
    class DistanceDetection {
    public:
        explicit DistanceDetection(CameraParamStereo _param, const int &_cameraDeviceNo = 0) : param(std::move(_param)),
                                                                                               cameraDeviceNo(
                                                                                                       _cameraDeviceNo) {
            numDisparities = static_cast<int>(setting["StereoBM_NumDisparities"].as_int64());
            blockSize = static_cast<int>(setting["StereoBM_BlockSize"].as_int64());
            uniquenessRatio = static_cast<int>(setting["StereoBM_UniquenessRatio"].as_int64());
            preFilterCap = static_cast<int>(setting["StereoBM_PreFilterCap"].as_int64());
        }

        void startContinuousDetection(const std::function<Type(DistanceDataFrame)> &subscriber) {
            Console::WriteLine("Preparing for detection...");
            cv::VideoCapture capture;
            // MSFS Option
#ifdef DISABLE_MSMF
            if (setting["Toggle_MSMF"].as_bool()) {
                Console::WriteLine("Open the camera while disabling Microsoft Media Foundation.");
                auto res = _putenv("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS=0");
            } else {
                Console::WriteLine("Open the camera by Microsoft Media Foundation.");
            }
#endif

#ifdef _WIN32
            capture.open(cameraDeviceNo);
#endif
#ifdef __LINUX__
            capture.open(cameraDeviceNo, cv::CAP_V4L2);
#endif
            // Camera trim
            capture.set(cv::CAP_PROP_FRAME_WIDTH, 3840);
            capture.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
            capture.set(cv::CAP_PROP_FPS, static_cast<int>(setting["FPS"].as_int64()));
            capture.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);

#ifndef NO_HIGUI
            const std::string winName = "Disparties";
            cv::namedWindow(winName);
            cv::resizeWindow(winName, 800, 600);
#endif

            int badFrame = 0;
            while (true) {
                cv::Mat frame;
                auto res = capture.read(frame);
                if (!res) {
                    if(badFrame>5){
                        Console::WriteLine("Bad frame at the limit, have you lost your USB connection?");
                    } else{
                        Console::WriteLine("Bad frame read, [ " + std::to_string(++badFrame) + " ]");
                    }
                }

            }


        }


    private:
        int cameraDeviceNo = 0;
        int numDisparities = 16;
        int blockSize = 6;
        int uniquenessRatio = 3;
        int preFilterCap = 16;
        CameraParamStereo param;

        auto getEpoch() {
            return std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
        }
    };

} // ArduCV

#endif //ARDUCV_DISTANCEDETECTION_H
