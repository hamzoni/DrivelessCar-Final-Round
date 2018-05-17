#ifndef LANE_H
#define LANE_H

#include <iostream>
#include "line.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"


class Lane{
public:
    int H;
    std::vector<cv::Point> cnt;
    Line line, line_full, line_far;
    Lane();
    Lane(std::vector<cv::Point>, int, int);

    void setLane(int);

    void setLineFar(int, int);

    void setLine(int, int);

    void changePoints(std::vector<cv::Point>);

private:
    void getLine(int, int, Line &, bool);
};
#endif // LANE_H
