#ifndef DEBUG_H
#define DEBUG_H

#include <opencv2/highgui/highgui.hpp>
#include "road.h"


namespace debug{
    extern cv::Scalar white;
    extern cv::Scalar sky_blue;

    extern cv::Scalar green;
    extern cv::Scalar yellow;
    extern cv::Scalar red;
    extern cv::Scalar blue;
    extern cv::Scalar violet;

    extern cv::Scalar black;
}

void drawPoints(cv::Mat &src,
                std::vector<cv::Point> &point,
                int radius,
                cv::Scalar color);

void draw(cv::Mat &mask,
          cv::Mat &roi,
          cv::Mat &bird,
          Road &road,
          std::vector<Lane> lanes,
          std::vector<cv::Point> &obj_cors);

void debugGreen(cv::Mat &mask,
                cv::Mat &mask_green,
                Road &road,
                std::vector<Lane> lanes);

void drawParabol(cv::Mat                  &src,
                 std::vector<cv::Point>    &points,
                 std::vector<double>       &parabol);

#endif // DEBUG_H
