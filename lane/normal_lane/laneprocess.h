#ifndef LANEPROCESS_H
#define LANEPROCESS_H

#include <iostream>
#include "lane.h"
#include "line.h"
#include "road.h"
#include "lineprocess.h"
#include "laneimageprocess.h"

bool canCombine(Lane &l1,
                Lane &l2);

void combineLanes(std::vector<Lane> &lanes,
                  int MIN_POINTS_AFTER_COMBINE);

void fillPoints(std::vector<cv::Point> &cnt, int start);

int ngaba(std::vector<Lane>    &lanes,
          Road                 &road,
          double SLOPE_MAX_LINE,
          int MIN_ANGLE_TWO_LINES,
          int DIS_ROAD_INJUNCTION);

void findLeft(std::vector<Lane> &lanes,
              Road              &road,
              int               MAX_DIS_LEFT,
              int MIN_Y);

void findRight(std::vector<Lane> &lanes,
               Road              &road,
               int               MAX_DIS_RIGHT,
               int MIN_Y);

void separateLeftRightByDistanceLine(std::vector<Lane>  &lanes,
                                     Road               &road);

void separateLeftRightByDistancePoint(std::vector<Lane> &lanes,
                                      Road              &road,
                                      int               MAX_DIS_LEFT,
                                      int               MAX_DIS_RIGHT,
                                      int               MIN_Y);

void separateLeftRightByDistancePointNoBird(std::vector<Lane>   &lanes,
                                            Road                &road,
                                            int                 MAX_DIS_LEFT,
                                            int                 MAX_DIS_RIGHT,
                                            int                 MIN_Y);

void adaptiveLeftRight(std::vector<Lane> &lanes,
                       Road              &road,
                       int               MAX_DIS_LEFT,
                       int               MAX_DIS_RIGHT,
                       int               MIN_Y,
                       int MAX_ANGLE_TWO_LANES,
                       int MIN_LEN_LANE,
                       double RATIO_POINTS_LR);

void calculateAngleRoad(Road &road);

void calculateFarRoad(Road  &road,
                      int   Y_BOT,
                      int   Y_TOP);

void genLine(Road &road);

bool processObject(Road                      &road,
                  std::vector<cv::Point>    obj_cors);

int removeNoiseLane(Road &r,
                    int MAX_ANGLE_TWO_LANES,
                    int MIN_LEN_LANE,
                    double RATIO_POINTS_LR,
                    bool isBird = false);

void curveLane(std::vector<Lane> lanes);

#endif // LANEPROCESS_H
