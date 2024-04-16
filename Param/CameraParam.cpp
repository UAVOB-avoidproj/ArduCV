//
// Created by Kira on 24-4-16.
//

#include "CameraParam.h"

namespace ArduCV {
    CameraParamStereo param;

    bool saveCalibrateData(const CameraParamStereo &data, const std::string &path) {
        try {
            cv::FileStorage file(path, cv::FileStorage::WRITE);

            file << "left_checkRange" << data.left.checkRange;
            file << "left_cameraMatrix" << data.left.cameraMatrix;
            file << "left_distCoeffs" << data.left.distCoeffs;
            file << "left_rotationVectors" << data.left.rotationVectors;
            file << "left_translationVectors" << data.left.translationVectors;
            file << "left_rms" << data.left.rms;

            file << "right_checkRange" << data.right.checkRange;
            file << "right_cameraMatrix" << data.right.cameraMatrix;
            file << "right_distCoeffs" << data.right.distCoeffs;
            file << "right_rotationVectors" << data.right.rotationVectors;
            file << "right_translationVectors" << data.right.translationVectors;
            file << "right_rms" << data.right.rms;

            file << "cameraRms" << data.rms;
            file << "cameraMatrixLeft" << data.cameraMatrixLeft;
            file << "distCoeffsLeft" << data.distCoeffsLeft;
            file << "cameraMatrixRight" << data.cameraMatrixRight;
            file << "distCoeffsRight" << data.distCoeffsRight;
            file << "rotationMatrix" << data.rotationMatrix;
            file << "translationMatrix" << data.translationMatrix;
            file << "essentialMatrix" << data.essentialMatrix;
            file << "fundamentalMatrix" << data.fundamentalMatrix;

            return true;
        }
        catch (cv::Exception &ex) {
            Console::Error(ex.what());
        }
        catch (std::exception &ex) {
            Console::Error(ex.what());
        }
        catch (...) {
            std::cout << "Unknown exception caught.\n";
            Console::Error("Unknown exception caught.");
        }
        return false;
    }

    std::pair<bool, CameraParamStereo> readCalibrateData(const std::string &path) {
        CameraParamStereo data;
        try {
            cv::FileStorage file(path, cv::FileStorage::READ);

            file["left_checkRange"] >> data.left.checkRange;
            file["left_cameraMatrix"] >> data.left.cameraMatrix;
            file["left_distCoeffs"] >> data.left.distCoeffs;
            file["left_rotationVectors"] >> data.left.rotationVectors;
            file["left_translationVectors"] >> data.left.translationVectors;
            file["left_rms"] >> data.left.rms;

            file["right_checkRange"] >> data.right.checkRange;
            file["right_cameraMatrix"] >> data.right.cameraMatrix;
            file["right_distCoeffs"] >> data.right.distCoeffs;
            file["right_rotationVectors"] >> data.right.rotationVectors;
            file["right_translationVectors"] >> data.right.translationVectors;
            file["right_rms"] >> data.right.rms;

            file["cameraRms"] >> data.rms;
            file["cameraMatrixLeft"] >> data.cameraMatrixLeft;
            file["distCoeffsLeft"] >> data.distCoeffsLeft;
            file["cameraMatrixRight"] >> data.cameraMatrixRight;
            file["distCoeffsRight"] >> data.distCoeffsRight;
            file["rotationMatrix"] >> data.rotationMatrix;
            file["translationMatrix"] >> data.translationMatrix;
            file["essentialMatrix"] >> data.essentialMatrix;
            file["fundamentalMatrix"] >> data.fundamentalMatrix;

        }
        catch (cv::Exception &ex) {
            std::cerr << ex.what() << "\n";
            Console::Error(ex.what());
        }
        catch (std::exception &ex) {
            std::cerr << ex.what() << "\n";
            Console::Error(ex.what());
        }
        catch (...) {
            Console::Error("Unknown exception caught.");
        }
        return std::make_pair(false, data);
    }

    void showCalibrateData(const CameraParamStereo &data) {
        std::cout << HIGHLIGHT "\nResult left: ##### \n" CLRST \
 << HIGHLIGHT "Check Range: \n" CLRST << data.left.checkRange << "\n"\
 << HIGHLIGHT "cameraMatrix: \n" CLRST << data.left.cameraMatrix << "\n"\
 << HIGHLIGHT "distCoeffs: \n" CLRST << data.left.distCoeffs << "\n"\
 << HIGHLIGHT "rvecs: \n" CLRST << data.left.rotationVectors << "\n"\
 << HIGHLIGHT "tvecs: \n" CLRST << data.left.translationVectors << "\n"\
 << HIGHLIGHT "rms: \n" CLRST << data.left.rms << "\n";

        std::cout << HIGHLIGHT "\nResult right: ##### \n" CLRST \
 << HIGHLIGHT "Check Range: \n" CLRST << data.right.checkRange << "\n"\
 << HIGHLIGHT "cameraMatrix: \n" CLRST << data.right.cameraMatrix << "\n"\
 << HIGHLIGHT "distCoeffs: \n" CLRST << data.right.distCoeffs << "\n"\
 << HIGHLIGHT "rvecs: \n" CLRST << data.right.rotationVectors << "\n"\
 << HIGHLIGHT "tvecs: \n" CLRST << data.right.translationVectors << "\n"\
 << HIGHLIGHT "rms: \n" CLRST << data.right.rms << "\n";

        std::cout << HIGHLIGHT "\nStereo calibration RMS: ##### \n" CLRST << data.rms << "\n"
                  << HIGHLIGHT "Left camera matrix: \n" CLRST << data.cameraMatrixLeft << "\n"\
 << HIGHLIGHT "Right camera matrix: \n" CLRST << data.cameraMatrixRight << "\n"\
 << HIGHLIGHT "Left dist coeffs: \n" CLRST << data.distCoeffsLeft << "\n"\
 << HIGHLIGHT "Right dist coeffs: \n" CLRST << data.distCoeffsRight << "\n" \
 << HIGHLIGHT "Rotation matrix: \n" CLRST << data.rotationMatrix << "\n"\
 << HIGHLIGHT "Translation matrix: \n" CLRST << data.translationMatrix << "\n"\
 << HIGHLIGHT "Essential matrix: \n" CLRST << data.essentialMatrix << "\n"\
 << HIGHLIGHT "Fundamental matrix: \n" CLRST << data.fundamentalMatrix << "\n";
    }
}