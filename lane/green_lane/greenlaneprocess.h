//
// Created by nc on 29/04/2018.
//

#ifndef CDS_GREENLANEPROCESS_H
#define CDS_GREENLANEPROCESS_H

#include "road.h"
#include "greenimageprocess.h"
#include "greconf.h"
#include "layer.h"

void fillPointsGreen(std::vector<cv::Point> &cnt, 
                     int start);

void averagePointsLane(Lane &src_lane,
                       int  NUM_AVERAGE_POINTS);

bool isRoadInjectionGreen(Lane &l1,
                          Lane &l2,
                          int DIS_ROAD_INJUNCTION);

void fillFullCirclePoints(std::vector<cv::Point> &cnt);

bool roadInject(std::vector<Lane>    &lanes,
                Road                 &road,
                double SLOPE_MAX_LINE,
                int MIN_ANGLE_TWO_LINES,
                int DIS_ROAD_INJUNCTION);

void removeNoiseLRGreen(Road &road);

void removeNoiseLaneGreen(std::vector<Lane> &lanes);

void tranformRoad(Road &src_road,
                  Road &dst_road,
                  bool avarage_point = false,
                  bool roadInject = false);

bool checkRoadInjectBeforeTranform(Road &src_road,
                                   Road &dst_road);

void lane2d2Bird(std::vector<Lane> &lanes,
                 double X_RATIO,
                 double Y_RATION);

void processContours(std::vector<std::vector<cv::Point>> &contours);

void combineLanesGreen(std::vector<Lane> &lanes,
                       int               MIN_POINTS_AFTER_COMBINE);

#endif //CDS_GREENLANEPROCESS_H
