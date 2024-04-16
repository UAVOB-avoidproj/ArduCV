//
// Created by Kira on 24-4-16.
//

#include "Calibration.h"

namespace ArduCV {


    SingleCalibration::SingleCalibration(int _chessBoardHeight, int _chessBoardWidth, float _squareSize) : squareSize(
            _squareSize) {
        boardSize.height = _chessBoardHeight;
        boardSize.width = _chessBoardWidth;
    }

    std::tuple<bool, Corners> SingleCalibration::findCorners(const cv::Mat &frame) {
        Corners corners;
        bool result = cv::findChessboardCorners(frame, boardSize, corners,
                                                cv::CALIB_CB_ADAPTIVE_THRESH /*| cv::CALIB_CB_FAST_CHECK */
                                                | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);
        return std::make_tuple(result, corners);
    }

    cv::Mat SingleCalibration::appendCorners(cv::Mat frame, Corners corners) {
        imageSize = frame.size();
        cv::cornerSubPix(frame, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001)); //## TAG
        cv::drawChessboardCorners(frame, boardSize, corners, true);
        imagePoints.push_back(corners);
        return frame;
    }

    std::tuple<bool, CameraMatrix, DistCoeffs, Rvecs, Tvecs, Rms, ImagePoints, ObjectPoints>
    SingleCalibration::calculate() {
        cv::Mat cameraMatrix, distCoeffs, rvecs, tvecs;
        std::vector<cv::Point3f> obj;
        for (int k = 0; k < boardSize.height; k++) {
            for (int l = 0; l < boardSize.width; l++) {
                obj.emplace_back(static_cast<float>(l) * squareSize, static_cast<float>(k) * squareSize, 0);
            }
        }
        objectPoints.push_back(obj);
        objectPoints.resize(imagePoints.size(), objectPoints[0]);
        double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
        bool result = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);
        return std::make_tuple(result, cameraMatrix, distCoeffs, rvecs, tvecs, rms, imagePoints, objectPoints);
    }

    InteractiveCameraCalibration::InteractiveCameraCalibration(int _chessBoardHeight, int _chessBoardWidth,
                                                               float _squareSize, int _cameraDeviceNo,
                                                               bool _useBinaryThreshold) :
            squareSize(_squareSize), cameraDeviceNo(_cameraDeviceNo), useBinaryThreshold(_useBinaryThreshold) {
        boardSize.height = _chessBoardHeight;
        boardSize.width = _chessBoardWidth;
    }

    InteractiveCameraCalibration::~InteractiveCameraCalibration() {
        cv::destroyAllWindows();
        if (capture.isOpened()) {
            capture.release();
        }
    }

    bool InteractiveCameraCalibration::startCalibration(CameraParamStereo &cameraParam) {
        auto captureTime = 0;
        auto badCaptureTime = 0;
        Console::WriteLine("Starting camera video capture.");
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
// Window trim
        cv::namedWindow("left", cv::WINDOW_GUI_EXPANDED);
        cv::namedWindow("right", cv::WINDOW_GUI_EXPANDED);
        cv::resizeWindow("left", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));
        cv::resizeWindow("right", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));
        cv::namedWindow("left_valid", cv::WINDOW_NORMAL);
        cv::namedWindow("right_valid", cv::WINDOW_NORMAL);
        cv::resizeWindow("left_valid", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));
        cv::resizeWindow("right_valid", static_cast<int>(setting["WinSizeWidth"].as_int64()),
                         static_cast<int>(setting["WinSizeHeight"].as_int64()));

        SingleCalibration leftCali(boardSize.height, boardSize.width, squareSize);
        SingleCalibration rightCali(boardSize.height, boardSize.width, squareSize);

        Console::WriteLine("Capture started. Press C to capture, E to execute calculation, Q to quit.");

        while (true) {
            cv::Mat frame;
            auto result = capture.read(frame);
            if (!result) {
                badCaptureTime++;
                Console::Error("Bad frame read, [ " + std::to_string(badCaptureTime) + " ]");
                if (badCaptureTime >= 5) {
                    Console::Error("Bad frame at the limit, have you lost your USB connection?");
                    return false;
                }
                continue;
            }
            cv::Size combinedImageSize = frame.size();
            auto leftImage = frame(cv::Rect(0, 0, combinedImageSize.width / 2, combinedImageSize.height));
            auto rightImage = frame(cv::Rect(combinedImageSize.width / 2, 0, combinedImageSize.width / 2,
                                             combinedImageSize.height));
            cv::Mat leftGray, rightGray;
            cv::cvtColor(leftImage, leftGray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(rightImage, rightGray, cv::COLOR_BGR2GRAY);
            if (useBinaryThreshold) {
                cv::threshold(leftGray, leftGray, 127, 255, cv::THRESH_BINARY);
                cv::threshold(rightGray, rightGray, 127, 255, cv::THRESH_BINARY);
            }
            cv::imshow("left", leftGray);
            cv::imshow("right", rightGray);
            auto res = cv::waitKey(1);
            if (res == 'C' || res == 'c') {        // Capture
                auto resultLeft = leftCali.findCorners(leftGray);
                auto resultRight = rightCali.findCorners(rightGray);
                if (std::get<0>(resultLeft) && std::get<0>(resultRight)) {
                    Console::WriteLine("Valid frame captured! Captured [ " + std::to_string(++captureTime) + " ]");
                    auto resImageLeft = leftCali.appendCorners(leftGray, std::get<1>(resultLeft));
                    auto resImageRight = rightCali.appendCorners(rightGray, std::get<1>(resultRight));
                    cv::imshow("left_valid", resImageLeft);
                    cv::imshow("right_valid", resImageRight);
                    cv::waitKey(1);
                } else {
                    Console::WriteLine("Invalid frame");
                    Console::WriteLine(
                            "Capture status: left -> " + std::to_string(std::get<0>(resultLeft)) + " , right -> " +
                            std::to_string(std::get<0>(resultRight)));
                    cv::drawChessboardCorners(leftGray, boardSize, std::get<1>(resultLeft),
                                              std::get<0>(resultLeft));
                    cv::imshow("left_valid", leftGray);
                    cv::drawChessboardCorners(rightGray, boardSize, std::get<1>(resultRight),
                                              std::get<0>(resultRight));
                    cv::imshow("right_valid", rightGray);
                    cv::waitKey(1);
                }
            } else if (res == 'Q' || res == 'q')        // Quit
            {
                cv::destroyAllWindows();
                capture.release();
                return false;
            } else if (res == 'E' || res == 'e')        // Execute camera calibration
            {
                std::chrono::steady_clock::time_point calibStart, leftDone, rightDone, stereoDone;
                cv::destroyAllWindows();
                capture.release();
                Console::WriteLine("Total [" + std::to_string(captureTime) + "] pictures captured.");
                Console::WriteLine("Starting calibration, it may take times...");
                Console::WriteLine("Left camera calibration...");
                calibStart = std::chrono::steady_clock::now();
                auto leftCalcRes = leftCali.calculate();
                leftDone = std::chrono::steady_clock::now();
                Console::WriteLine("Elapse time: " + std::to_string(
                        std::chrono::duration_cast<std::chrono::seconds>(leftDone - calibStart).count()));
                Console::WriteLine("Right camera calibration...");
                auto rightCalcRes = rightCali.calculate();
                rightDone = std::chrono::steady_clock::now();
                Console::WriteLine("Elapse time: " + std::to_string(
                        std::chrono::duration_cast<std::chrono::seconds>(rightDone - leftDone).count()));

                auto leftImagePoints = std::get<6>(leftCalcRes);
                auto leftCheckRange = std::get<0>(leftCalcRes);
                auto leftCameraMatrix = std::get<1>(leftCalcRes);
                auto leftDistCoeffs = std::get<2>(leftCalcRes);
                auto leftRvecs = std::get<3>(leftCalcRes);
                auto leftTvecs = std::get<4>(leftCalcRes);
                auto leftRms = std::get<5>(leftCalcRes);
                auto rightImagePoints = std::get<6>(rightCalcRes);
                auto rightCheckRange = std::get<0>(rightCalcRes);
                auto rightCameraMatrix = std::get<1>(rightCalcRes);
                auto rightDistCoeffs = std::get<2>(rightCalcRes);
                auto rightRvecs = std::get<3>(rightCalcRes);
                auto rightTvecs = std::get<4>(rightCalcRes);
                auto rightRms = std::get<5>(rightCalcRes);

// Save param
                cameraParam.left.checkRange = leftCheckRange;
                cameraParam.left.cameraMatrix = leftCameraMatrix;
                cameraParam.left.distCoeffs = leftDistCoeffs;
                cameraParam.left.rotationVectors = leftRvecs;
                cameraParam.left.translationVectors = leftTvecs;
                cameraParam.left.rms = leftRms;
                cameraParam.right.checkRange = rightCheckRange;
                cameraParam.right.cameraMatrix = rightCameraMatrix;
                cameraParam.right.distCoeffs = rightDistCoeffs;
                cameraParam.right.rotationVectors = rightRvecs;
                cameraParam.right.translationVectors = rightTvecs;
                cameraParam.right.rms = rightRms;

// Stereo calibration
                cv::Mat R, T, E, F;
                cv::Size newImageSize;
                cv::Rect validRoi[2];
                cv::Size imageSize(combinedImageSize.width / 2, combinedImageSize.height);
                Console::WriteLine("Stereo calibration...");

// Extend objectPoints
                std::vector<cv::Point3f> obj;
                for (int k = 0; k < boardSize.height; k++) {
                    for (int l = 0; l < boardSize.width; l++) {
                        obj.emplace_back(static_cast<float>(l) * squareSize, static_cast<float>(k) * squareSize, 0);
                    }
                }
                objectPoints.push_back(obj);
                objectPoints.resize(captureTime, objectPoints[0]);
                auto rms = cv::stereoCalibrate(
                        objectPoints, leftImagePoints,
                        rightImagePoints, leftCameraMatrix,
                        leftDistCoeffs, rightCameraMatrix,
                        rightDistCoeffs, imageSize, R, T, E, F,
                        cv::CALIB_USE_INTRINSIC_GUESS,
                        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                         30, 1e-6)
                );
                stereoDone = std::chrono::steady_clock::now();
                Console::WriteLine("Elapse time: " + std::to_string(
                        std::chrono::duration_cast<std::chrono::seconds>(stereoDone - rightDone).count()));
                Console::WriteLine("Total elapse time: " + std::to_string(
                        std::chrono::duration_cast<std::chrono::seconds>(stereoDone - calibStart).count()));

// Save param
                cameraParam.rms = rms;
                cameraParam.cameraMatrixLeft = leftCameraMatrix;
                cameraParam.cameraMatrixRight = rightCameraMatrix;
                cameraParam.distCoeffsLeft = leftDistCoeffs;
                cameraParam.distCoeffsRight = rightDistCoeffs;
                cameraParam.rotationMatrix = R;
                cameraParam.translationMatrix = T;
                cameraParam.essentialMatrix = E;
                cameraParam.fundamentalMatrix = F;
                lastCalibrate = cameraParam;

                showCalibrateData(cameraParam);
                Console::WriteLine("Calibration done with rms " + std::to_string(cameraParam.rms));
                Console::WriteLine("Left rms " + std::to_string(cameraParam.left.rms));
                Console::WriteLine("Right rms " + std::to_string(cameraParam.right.rms));

                if (cameraParam.rms < 1 && cameraParam.left.rms < 1 && cameraParam.right.rms < 1) {
                    Console::WriteLine("RMS check ok");
                } else {
                    Console::WriteLine("RMS is too high, re-calibration needed.");
                }
                param = lastCalibrate;
                saveCalibrateData(param);
                Console::WriteLine("Param Saved");
                return true;
            }
        }
    }


} // ArduCV