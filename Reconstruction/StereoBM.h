//
// Created by Kira on 24-4-17.
//

#ifndef ARDUCV_STEREOBM_H
#define ARDUCV_STEREOBM_H

#include <opencv2/opencv.hpp>
#include <utility>
#include "../Types/Types.h"
#include "../Tools/Tools.h"

namespace ArduCV {

    class StereoBMCompute {
    public:
        StereoBMCompute();

        StereoBMCompute(Roi ROI1, Roi ROI2, cv::Mat Q);

        void SetParam(Roi Roi1, Roi Roi2, cv::Mat q);

        std::tuple<cv::Mat,cv::Mat,cv::Mat> Compute(const cv::Mat &Left, const cv::Mat &Right, int numDisparities, int blockSize, int preFilterCap,
                     int uniquenessRatio);
        std::tuple<cv::Mat,cv::Mat,cv::Mat> Compute(const cv::UMat &Left, const cv::UMat &Right, int numDisparities, int blockSize, int preFilterCap,
                     int uniquenessRatio);

    private:
        cv::Mat disp, disp8, _3dImage, Q;
        Roi ROI1, ROI2;
    };

}


#endif //ARDUCV_STEREOBM_H
