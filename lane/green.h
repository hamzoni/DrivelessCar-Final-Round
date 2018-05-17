//
// Created by nc on 13/04/2018.
//

#ifndef CDS_GREEN_H
#define CDS_GREEN_H

#include "opencv2/highgui/highgui.hpp"
#include "road.h"
#include "greenlaneprocess.h"

void processRoadGreen(cv::Mat &src,
                      Road &road);

void processLayerGridA(Road &road,
                       std::vector<std::vector<cv::Point> > &contours,
                       double SLOPE_MAX_LINE_INJUNCTION,
                       int ANGLE_LINES_INJECTION,
                       int DIS_ROAD_INJUNCTION,
                       bool findLanes = false);

void detectLaneGreen(cv::Mat    &src,
                     Road       &road);

void detectRoadInject(cv::Mat &src,
                      Road &road,
                      double SLOPE_MAX_LINE_INJUNCTION,
                      int ANGLE_LINES_INJECTION,
                      int DIS_ROAD_INJUNCTION);
#endif //CDS_GREEN_H
