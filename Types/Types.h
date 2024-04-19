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

    typedef cv::Rect Roi;
    typedef cv::Mat Map;

    typedef std::vector<cv::Point2f> Corners;
    typedef std::vector<std::vector<cv::Point3f>> ObjectPoints;
    typedef	std::vector<std::vector<cv::Point2f>> ImagePoints;

    /**
     * @brief Recommended Flight Path Vector
     */
    struct RecVec{
        double Xdirection;
        double Ydirection;
        double Angle;
    };
    /**
     * @brief The data frame that subscriber will receive
     */
    struct DistanceDataFrame {
        // epoch time
        std::chrono::duration<long long, std::ratio<1, 1000000>> epoch;
        // the distance matrix [row][col]
        std::vector<std::vector<double>> matrix;
        // the percentage which inside thres [row][col]
        std::vector<std::vector<double>> nearPercentage;
        // recommend forward vector
        RecVec recommendVector;
        // duration in microsecond
        long long duration;
        // respond speed
        double fps;
    };
}


#endif //ARDUCV_TYPES_H
