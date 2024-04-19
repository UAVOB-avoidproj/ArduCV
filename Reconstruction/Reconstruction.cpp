//
// Created by Kira on 24-4-16.
//

#include "Reconstruction.h"

#include <utility>

namespace ArduCV {

    cv::Mat disp, disp8;
    cv::Mat _3dImage;

    Reconstruction::Reconstruction(CameraParamStereo _param, const int &_cameraDeviceNo) : param(std::move(_param)),
                                                                                               cameraDeviceNo(
                                                                                                       _cameraDeviceNo) {
        numDisparities = static_cast<int>(setting["StereoBM_NumDisparities"].as_int64());
        blockSize = static_cast<int>(setting["StereoBM_BlockSize"].as_int64());
        uniquenessRatio = static_cast<int>(setting["StereoBM_UniquenessRatio"].as_int64());
        preFilterCap = static_cast<int>(setting["StereoBM_PreFilterCap"].as_int64());
        parallel = setting["Toggle_Partitional_Parallel"].as_bool();
    }
    Reconstruction::~Reconstruction() {
        cv::destroyAllWindows();
        if (capture.isOpened()) {
            capture.release();
        }
    }
    void Reconstruction::InteractiveReconstruction() {
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
        cv::namedWindow("left", cv::WINDOW_GUI_NORMAL);
        cv::namedWindow("right", cv::WINDOW_GUI_EXPANDED);
        cv::resizeWindow("left", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));
        cv::resizeWindow("right", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));
        cv::namedWindow("disparity", cv::WINDOW_NORMAL);
        cv::resizeWindow("disparity", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));
        cv::namedWindow("param", cv::WINDOW_NORMAL);
        cv::resizeWindow("param", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));

        int badCaptureTime = 0;
        cv::UMat frame;
        bool rtBM = setting["Toggle_RT_StereoBM"].as_bool();

        // Create param slide bar
        // It must be an odd number >=1 . Normally, it should be somewhere in the 3..11 range.
        cv::createTrackbar("Block Size:\n", "param", &blockSize, 12);
        // Normally, a value within the 5-15 range is good enough.
        cv::createTrackbar("Uniqueness Ratio:\n", "param", &uniquenessRatio, 16);
        // The value is always greater than zero. This parameter must be divisible by 16
        cv::createTrackbar("Num Disparities:\n", "param", &numDisparities, 64);
        // Gray sensitivity reduction. 5~15 (Condition good, bright) to 20~63(Noisy, dim)
        cv::createTrackbar("Pre Filter Cap:\n", "param", &numDisparities, 48);

        cv::setMouseCallback("disparity", onClick, &param);

        if (!rtBM) {
            Console::WriteLine("Non real time mode, press 'C' to capture");
        }

        while (true) {
            auto result = capture.read(frame);
            if (!result) {
                badCaptureTime++;
                Console::WriteLine("Bad frame read, [ " + std::to_string(badCaptureTime) + " ]");
                if (badCaptureTime >= 5) {
                    Console::WriteLine("Bad frame at the limit, have you lost your USB connection?");
                    return;
                }
                continue;
            }
            cv::Size combinedImageSize = frame.size();
            auto leftImage = frame(cv::Rect(0, 0, combinedImageSize.width / 2, combinedImageSize.height));
            auto rightImage = frame(cv::Rect(combinedImageSize.width / 2, 0, combinedImageSize.width / 2,
                                             combinedImageSize.height));
            cv::UMat leftGray, rightGray, leftRectify, rightRectify;
            cv::cvtColor(leftImage, leftGray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(rightImage, rightGray, cv::COLOR_BGR2GRAY);

            // StereoRectify
            if (!rectify) {
                cv::Size imageSize(combinedImageSize.width / 2, combinedImageSize.height);
                StereoRectify(imageSize);
                rectify = true;
            }
            cv::remap(leftGray, leftRectify, mapLx, mapLy, cv::INTER_LINEAR);
            cv::remap(rightGray, rightRectify, mapRx, mapRy, cv::INTER_LINEAR);
            cv::imshow("left", leftRectify);
            cv::imshow("right", rightRectify);

            // StereoBM
            if (rtBM) {
                std::pair<cv::Mat, cv::Mat> ret;
                if (parallel) {
                    ret = StereoMatchBMPartitionalParallel(leftRectify, rightRectify);
                } else {
                    ret = StereoMatchBM(leftRectify, rightRectify);
                }
                cv::imshow("disparity", ret.second);
            }
            auto key = cv::waitKey(1);
            if (key == 'q' || key == 'Q') {
                capture.release();
                cv::destroyAllWindows();
                setting["StereoBM_NumDisparities"] = numDisparities;
                setting["StereoBM_BlockSize"] = blockSize;
                setting["StereoBM_UniquenessRatio"] = uniquenessRatio;
                setting["StereoBM_PreFilterCap"] = preFilterCap;
                return;
            } else if (!rtBM && (key == 'c' || key == 'C')) {
                std::pair<cv::Mat, cv::Mat> ret;
                if (parallel) {
                    ret = StereoMatchBMPartitionalParallel(leftRectify, rightRectify);
                } else {
                    ret = StereoMatchBM(leftRectify, rightRectify);
                }
                cv::imshow("disparity", ret.second);
            }
        }
    }

    std::pair<cv::Mat, cv::Mat> Reconstruction::StereoMatchBM(const cv::UMat& left, const cv::UMat& right) {
        auto stereoBM = cv::StereoBM::create(128, 9);
        stereoBM->setMinDisparity(0);
        stereoBM->setNumDisparities(numDisparities * 16 + 16);
        stereoBM->setBlockSize(2 * blockSize + 5);
        stereoBM->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
        stereoBM->setPreFilterSize(5);
        stereoBM->setPreFilterCap(preFilterCap);
        stereoBM->setROI1(ROI1);
        stereoBM->setROI2(ROI2);
        stereoBM->setTextureThreshold(10);
        stereoBM->setUniquenessRatio(uniquenessRatio);
        stereoBM->setSpeckleWindowSize(100);
        stereoBM->setSpeckleRange(32);
        stereoBM->setDisp12MaxDiff(-1);
        stereoBM->compute(left, right, disp);
        disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16) * 16.));
        cv::reprojectImageTo3D(disp8, _3dImage, Q, true);
        _3dImage *= 16;
        return std::make_pair(disp, disp8);
    }

    std::pair<cv::Mat, cv::Mat> Reconstruction::StereoMatchBMPartitionalParallel(cv::UMat left, cv::UMat right) {
        auto ParallelCol = static_cast<int>(setting["Partitional_Parallel_Cols"].as_int64());
        auto ParallelRow = static_cast<int>(setting["Partitional_Parallel_Rows"].as_int64());
        std::mutex mtx;
        std::vector<cv::Ptr<cv::StereoBM>> bmSet;
        int rows = left.rows / ParallelRow;
        int cols = left.cols / ParallelCol;
        struct SpiltResult {
            SpiltResult() = default;
            SpiltResult(int _part, cv::UMat _output) : part(_part), output(std::move(_output)) {};
            int part{};
            cv::UMat output;
        };
        cv::Mat combind(left.size(), CV_16S);
        std::deque<SpiltResult> dispQ;
        // Parallel Create StreroBMs
        cv::parallel_for_(cv::Range(0, ParallelCol * ParallelRow), [&](const cv::Range &range) {
            auto stereoBM = cv::StereoBM::create(128, 9);
            stereoBM->setMinDisparity(0);
            stereoBM->setNumDisparities(numDisparities * 16 + 16);
            stereoBM->setBlockSize(2 * blockSize + 5);
            stereoBM->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
            stereoBM->setPreFilterSize(5);
            stereoBM->setPreFilterCap(preFilterCap);
            stereoBM->setROI1(ROI1);
            stereoBM->setROI2(ROI2);
            stereoBM->setTextureThreshold(10);
            stereoBM->setUniquenessRatio(uniquenessRatio);
            stereoBM->setSpeckleWindowSize(100);
            stereoBM->setSpeckleRange(32);
            stereoBM->setDisp12MaxDiff(-1);
            mtx.lock();
            bmSet.push_back(stereoBM);
            mtx.unlock();
        });

        // Parallel Compute StreroBM
        cv::parallel_for_(cv::Range(0, ParallelCol * ParallelRow), [&](const cv::Range &range) {
            for (int i = range.start; i < range.end; ++i) {
                auto stereoBM = bmSet[i];
                int rowIdx = i / ParallelRow;
                int colIdx = i % ParallelCol;
                cv::Rect roi(colIdx * cols, rowIdx * rows, cols, rows);
                auto spiltLeft = left(roi);
                auto spiltRight = right(roi);
                cv::UMat output;
                stereoBM->compute(spiltLeft, spiltRight, output);
                mtx.lock();
                dispQ.emplace_back(i, output);
                mtx.unlock();
            }
        });
        // Combind result of each thread
        do {
            SpiltResult sp;
            sp = dispQ.front();
            dispQ.pop_front();
            int rowIdx = sp.part / ParallelRow;
            int colIdx = sp.part % ParallelCol;
            cv::Rect roi(colIdx * cols, rowIdx * rows, cols, rows);
            auto part = combind(roi);
            sp.output.copyTo(part);
        } while (!dispQ.empty());
        disp = combind;
        disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16) * 16.));
        cv::reprojectImageTo3D(disp8, _3dImage, Q, true);
        _3dImage *= 16;
        return std::make_pair(disp, disp8);
    }

    void Reconstruction::onClick(int event, int x, int y, int z, void *userdata) {
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

    void Reconstruction::StereoRectify(const cv::Size &imageSize) {
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

}