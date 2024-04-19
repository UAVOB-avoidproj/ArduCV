//
// Created by Kira on 24-4-16.
//

#include "DistanceDetection.h"

namespace ArduCV {
    DistanceDetection::DistanceDetection(CameraParamStereo _param, const int &_cameraDeviceNo) : camParam(
            std::move(_param)),
                                                                                                 cameraDeviceNo(
                                                                                                         _cameraDeviceNo) {
        numDisparities = static_cast<int>(setting["StereoBM_NumDisparities"].as_int64());
        blockSize = static_cast<int>(setting["StereoBM_BlockSize"].as_int64());
        uniquenessRatio = static_cast<int>(setting["StereoBM_UniquenessRatio"].as_int64());
        preFilterCap = static_cast<int>(setting["StereoBM_PreFilterCap"].as_int64());
    }

    DistanceDetection::~DistanceDetection() = default;


    void DistanceDetection::startContinuousDetection(const std::function<void(DistanceDataFrame)> &subscriber,
                                                     int nearThres) {
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
        capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<int>(setting["CameraResolutionWidth"].as_int64()));
        capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<int>(setting["CameraResolutionHeight"].as_int64()));
        capture.set(cv::CAP_PROP_FPS, static_cast<int>(setting["FPS"].as_int64()));
        capture.set(cv::CAP_PROP_FOURCC, cv::CAP_OPENCV_MJPEG);
// #define NO_HIGUI
#ifndef NO_HIGUI
        const std::string winName = "Disparties";
        cv::namedWindow(winName);
        cv::resizeWindow(winName, 800, 600);
#endif

        StereoRectify stereoRectify;
        StereoBMCompute stereoBmCompute;
        bool rectify = false;
        int badFrame = 0;

        auto baseline = cv::norm(camParam.translationMatrix) / 1000;
        auto focal_length = camParam.cameraMatrixLeft.at<double>(0, 0);

        while (true) {
            auto start = std::chrono::high_resolution_clock::now();
            cv::Mat frame;
            auto res = capture.read(frame);
            if (!res) {
                if (badFrame > 5) {
                    Console::WriteLine("Bad frame at the limit, have you lost your USB connection?");
                } else {
                    Console::WriteLine("Bad frame read, [ " + std::to_string(++badFrame) + " ]");
                }
            }

            // Spilt Image
            cv::Size combinedImageSize = frame.size();
            auto leftImage = frame(cv::Rect(0, 0, combinedImageSize.width / 2, combinedImageSize.height));
            auto rightImage = frame(cv::Rect(combinedImageSize.width / 2, 0, combinedImageSize.width / 2,
                                             combinedImageSize.height));
            // Gray Scale
            cv::UMat leftGray, rightGray, leftRectify, rightRectify;
            cv::cvtColor(leftImage, leftGray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(rightImage, rightGray, cv::COLOR_BGR2GRAY);

            // StereoRectify
            if (!rectify) {
                cv::Size imageSize(combinedImageSize.width / 2, combinedImageSize.height);
                stereoRectify.RectifyCalculate(camParam, imageSize);
                stereoBmCompute.SetParam(stereoRectify.ROI1, stereoRectify.ROI2, stereoRectify.Q);
                rectify = true;
            }

            std::tie(leftRectify, rightRectify) = stereoRectify.Remap(leftGray, rightGray);

            // StereoBM
            std::tie(disp, disp8, _3dImage) = stereoBmCompute.Compute(leftRectify, rightRectify, numDisparities,
                                                                      blockSize, preFilterCap, uniquenessRatio);

            // Analysis
            int cols = disp.cols;   // --
            int rows = disp.rows;   // ||
            int colOffset = 250;         // Cut left blank
            int colRoiX = colOffset + (cols - colOffset) / 3;       // ||
            int colRoiY = colOffset + (cols - colOffset) / 3 * 2;   // ||
            int rowRoiX = rows / 3;       // __
            int rowRoiY = rows / 3 * 2;   // __

#ifndef NO_HIGUI
            cv::Mat image;
            cv::cvtColor(disp8, image, cv::COLOR_GRAY2BGR);
            cv::line(image, cv::Point(colOffset, 0), cv::Point(colOffset, rows), cv::Scalar(0, 0, 255), 2,
                     cv::LINE_8);
            cv::line(image, cv::Point(colRoiX, 0), cv::Point(colRoiX, rows), cv::Scalar(255, 255, 0), 2,
                     cv::LINE_8);
            cv::line(image, cv::Point(colRoiY, 0), cv::Point(colRoiY, rows), cv::Scalar(255, 255, 0), 2,
                     cv::LINE_8);
            cv::line(image, cv::Point(0, rowRoiX), cv::Point(cols, rowRoiX), cv::Scalar(0, 255, 255), 2,
                     cv::LINE_8);
            cv::line(image, cv::Point(0, rowRoiY), cv::Point(cols, rowRoiY), cv::Scalar(0, 255, 255), 2,
                     cv::LINE_8);
            cv::imshow(winName, image);
            cv::waitKey(1);
#endif

            std::vector<std::vector<double>> result;
            result.resize(3, std::vector<double>(3));
            std::vector<std::vector<double>> nearPercent;
            nearPercent.resize(3, std::vector<double>(3));
            std::mutex lock;
            cv::parallel_for_(cv::Range(0, 9), [&](const cv::Range &range) {
                for (int i = range.start; i < range.end; i++) { // 9 blocks
                    int xPart = i % 3; // [0,1,2]
                    int yPart = i / 3; // [0,1,2]
//                        cv::Range col((cols / 3) * xPart, (cols / 3) * (xPart + 1));
//                        cv::Range row((rows / 3) * yPart, (rows / 3) * (yPart + 1));
                    cv::Range col, row;
                    switch (i) {
                        case 0: {
                            col.start = 0;
                            col.end = colRoiX;
                            row.start = 0;
                            row.end = rowRoiX;
                            break;
                        }
                        case 1: {
                            col.start = colRoiX;
                            col.end = colRoiY;
                            row.start = 0;
                            row.end = rowRoiX;
                            break;
                        }
                        case 2: {
                            col.start = colRoiY;
                            col.end = cols;
                            row.start = 0;
                            row.end = rowRoiX;
                            break;
                        }
                        case 3: {
                            col.start = 0;
                            col.end = colRoiX;
                            row.start = rowRoiX;
                            row.end = rowRoiY;
                            break;
                        }
                        case 4: {
                            col.start = colRoiX;
                            col.end = colRoiY;
                            row.start = rowRoiX;
                            row.end = rowRoiY;
                            break;
                        }
                        case 5: {
                            col.start = colRoiY;
                            col.end = cols;
                            row.start = rowRoiX;
                            row.end = rowRoiY;
                            break;
                        }
                        case 6: {
                            col.start = 0;
                            col.end = colRoiX;
                            row.start = rowRoiY;
                            row.end = rows;
                            break;
                        }
                        case 7: {
                            col.start = colRoiX;
                            col.end = colRoiY;
                            row.start = rowRoiY;
                            row.end = rows;
                            break;
                        }
                        case 8: {
                            col.start = colRoiY;
                            col.end = cols;
                            row.start = rowRoiY;
                            row.end = rows;
                            break;
                        }
                        default:
                            throw std::exception("Fuck you!");
                    }

                    std::vector<double> dotRangeList;
                    long long nearCount = 0;
                    cv::parallel_for_(row, [&](const cv::Range &rowSpilt) {           // Each row
                        for (int y = rowSpilt.start; y < rowSpilt.end; y++) {
                            cv::parallel_for_(col, [&](const cv::Range &colSpilt) {     // Each col
                                for (int x = colSpilt.start; x < colSpilt.end; x++) {
                                    auto dispVal = (disp.at<uint16_t>(y, x)) / 16;
                                    auto depth = (baseline * focal_length) / (dispVal) * 100;   // cm
                                    if (depth > 10 && depth < 600) {
                                        dotRangeList.push_back(depth);
                                        if (depth < nearThres) {
                                            nearCount++;
                                        }
                                    }

                                }
                            });
                        }
                    });
                    // auto res = std::accumulate(dotRangeList.begin(), dotRangeList.end(), 0.0);
                    // result[xPart][yPart] = res / static_cast<double>(dotRangeList.size());
                    if (!dotRangeList.empty()) {
                        double sum = tbb::parallel_reduce(
                                tbb::blocked_range<std::vector<double>::iterator>(
                                        dotRangeList.begin(), dotRangeList.end()
                                ), 0, [](tbb::blocked_range<std::vector<double>::iterator> const &r, double init) {
                                    return std::accumulate(r.begin(), r.end(), init);
                                }, std::plus());
                        nearPercent[yPart][xPart] =
                                static_cast<double>(nearCount) / static_cast<double>(dotRangeList.size());
                        result[yPart][xPart] = sum / static_cast<double>(dotRangeList.size());
                    } else {
                        nearPercent[yPart][xPart] = 0;
                        result[yPart][xPart] = -1;
                    }
                }
            });
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            auto fps = 1000000.0 / static_cast<double>(duration.count());

            RecVec recVec{};
            // auto npSumXL = nearPercent[0][0]+nearPercent[1][0]+nearPercent[2][0];
            // auto npSumXR = nearPercent[0][2]+nearPercent[1][2]+nearPercent[2][2];
            recVec.Xdirection =
                    // Left distance
                    (-result[0][0] * 0.25) +
                    (-result[1][0] * 0.50) +
                    (-result[2][0] * 0.25) +
                    // Right distance
                    (result[0][2] * 0.25) +
                    (result[1][2] * 0.50) +
                    (result[2][2] * 0.25);

            // auto npSumYU = nearPercent[0][0]+nearPercent[0][1]+nearPercent[0][2];
            // auto npSumYD = nearPercent[2][0]+nearPercent[2][1]+nearPercent[2][2];

            recVec.Ydirection =
                    // Up distance
                    (result[0][0] * 0.25) +
                    (result[0][1] * 0.50) +
                    (result[0][2] * 0.25) +
                    // Down distance
                    (-result[2][0] * 0.25) +
                    (-result[2][1] * 0.50) +
                    (-result[2][2] * 0.25);

            // Recommend Angel
            recVec.Angle = std::atan2(recVec.Xdirection, recVec.Ydirection) * static_cast<double>(180.0 / std::_Pi);

            DistanceDataFrame send{};
            send.epoch = getEpoch();
            send.matrix = result;
            send.duration = duration.count();
            send.nearPercentage = nearPercent;
            send.fps = fps;
            send.recommendVector = recVec;
            subscriber(send);

#ifdef DISDET_RES_WRITE
            Console::WriteLine("Result:");
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    Console::Write(std::to_string(static_cast<int>(result[i][j])) + " ");
                }
                Console::WriteLine("");
            }
            Console::WriteLine("FPS: " + std::to_string(fps) + " (" + std::to_string(duration.count() / 1000) + "ms)");
#endif
            if (cancellationToken.load()) {
#ifndef NO_HIGUI
                cv::destroyAllWindows();
#endif
                return;
            }
        }
    }

    std::chrono::duration<long long, std::ratio<1, 1000000>> DistanceDetection::getEpoch() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch());
    }

    void DistanceDetection::cancelTask() {
        cancellationToken.store(true);
    }
} // ArduCV