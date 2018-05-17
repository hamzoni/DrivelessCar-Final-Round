//
// Created by nc on 06/05/2018.
//

#ifndef CDS_STATUS_H
#define CDS_STATUS_H

#include "road.h"
#include "debug.h"
#include "laneprocess.h"
#include "green.h"
#include "fuzzylogic.h"

namespace status{
    extern bool NGABA;
    extern bool LEFT;

    extern bool LOST_LANE;
    extern bool BOTTLE;
    extern bool OBJ;
    extern bool STOP;

    extern double ANGLE;
    extern int SPEED;
    extern std::vector<cv::Point> obj_cors;
    extern int TRAFFIC;
}

namespace pstatus{
    extern Road road;
}

void getMaskWhite(cv::Mat &gray,
                  cv::Mat &hsv,
                  cv::Mat &dst,
                  cv::Mat &green,
                  std::vector<std::vector<cv::Point>> &hull);

void calculateAngle(Road &road);

void updateObj(cv::Rect &box_obj,
               int X_RATIO,
               int Y_RATIO);

void showStatus();


#endif //CDS_STATUS_H
