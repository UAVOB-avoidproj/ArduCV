//
// Created by Kira on 24-4-16.
//

// Pre defines
#ifdef _WIN32
#define DISABLE_MSMF	// Disable Microsoft Media Foundation feature on windows can let capture initialize faster
#endif

#ifdef __linux__
#define __LINUX__
#endif

#ifndef ARDUCV_TYPES_H
#define ARDUCV_TYPES_H

#include <boost/json.hpp>
#include <opencv2/opencv.hpp>

namespace ArduCV{

    typedef boost::json::object Setting;

    typedef cv::Mat CameraMatrix;
    typedef cv::Mat DistCoeffs;
    typedef cv::Mat Rvecs;
    typedef cv::Mat Tvecs;
    typedef double Rms;

    typedef cv::Mat Rmat;
    typedef cv::Mat Tmat;
    typedef cv::Mat Emat;
    typedef cv::Mat Fmat;

    typedef std::vector<cv::Point2f> Corners;
    typedef std::vector<std::vector<cv::Point3f>> ObjectPoints;
    typedef	std::vector<std::vector<cv::Point2f>> ImagePoints;
}


#endif //ARDUCV_TYPES_H
