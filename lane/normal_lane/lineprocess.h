#ifndef LINEPROCESS_H
#define LINEPROCESS_H

#include "line.h"
#include "pointsprocess.h"


bool intersection(cv::Point &R,
                  Line &l1,
                  Line &l2);

bool checkInRegion(cv::Point &p,
                   int W,
                   int H);


bool getPointFromDis(Line &line,
                     cv::Point &anchorPoint,
                     cv::Point &src_point,
                     int DIS);

#endif // LINEPROCESS_H
