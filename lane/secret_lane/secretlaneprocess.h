//
// Created by nc on 29/04/2018.
//

#ifndef CDS_SECRETGREENLANEPROCESS_H
#define CDS_SECRETGREENLANEPROCESS_H

#include "road.h"
#include "greconf.h"
#include "layer.h"
#include "debug.h"
#include "greenlaneprocess.h"
#include "seconf.h"

void processContours(std::vector<std::vector<cv::Point>> &contours);

void processRoadGreenSe(cv::Mat &src,
                      Road &road);

int removeNoiseLaneSe(Road &r, bool isBird);

void processLayerGridSe(Road &road,
                       std::vector<std::vector<cv::Point> > &contours,
                       double SLOPE_MAX_LINE_INJUNCTION,
                       int ANGLE_LINES_INJECTION,
                       int DIS_ROAD_INJUNCTION,
                        bool findRoadInj = false,
                       bool findLanes = false);

#endif //CDS_SECRETGREENLANEPROCESS_H
