//
// Created by Kira on 24-4-17.
//

#include "StereoBM.h"

namespace ArduCV {
    StereoBMCompute::StereoBMCompute() = default;

    StereoBMCompute::StereoBMCompute(Roi ROI1, Roi ROI2, cv::Mat Q) : ROI1(ROI1), ROI2(ROI2), Q(std::move(Q)) {

    }

    void StereoBMCompute::SetParam(Roi Roi1, Roi Roi2, cv::Mat q)
        {
            ROI1 = Roi1;
            ROI2 = Roi2;
            Q=std::move(q);
        }

    std::tuple<cv::Mat, cv::Mat, cv::Mat>
    StereoBMCompute::Compute(const cv::Mat &Left, const cv::Mat &Right, int numDisparities, int blockSize,
                             int preFilterCap,
                             int uniquenessRatio) {
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
        stereoBM->compute(Left, Right, disp);
        disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16) * 16.));
        cv::reprojectImageTo3D(disp8, _3dImage, Q, true);
        _3dImage *= 16;
        return std::make_tuple(disp, disp8, _3dImage);
    }

    std::tuple<cv::Mat, cv::Mat, cv::Mat>
    StereoBMCompute::Compute(const cv::UMat &Left, const cv::UMat &Right, int numDisparities, int blockSize,
                             int preFilterCap,
                             int uniquenessRatio) {
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
        stereoBM->compute(Left, Right, disp);
        disp.convertTo(disp8, CV_8U, 255 / ((numDisparities * 16 + 16) * 16.));
        cv::reprojectImageTo3D(disp8, _3dImage, Q, true);
        _3dImage *= 16;
        return std::make_tuple(disp, disp8, _3dImage);
    }
}
