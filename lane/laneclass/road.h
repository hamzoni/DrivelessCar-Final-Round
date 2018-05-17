#ifndef ROAD_H
#define ROAD_H

#include "lane.h"
#include <opencv2/highgui/highgui.hpp>
#include "config.h"

class Road{
public:
    int W, H;

    cv::Point pointBot, pointTop;
    cv::Point pointBotFar, pointTopFar;
    cv::Point road_inject;

    Lane right;
    Lane left;
    Lane middle;

    bool hasRight, hasLeft, hasMiddle, hasRoadInject, turn_left, fullLaneWhite;

    double angle, angleFar;
    Road();
    Road(int, int);

    void setRight(Lane);
    void setLeft(Lane);
    void setMiddle(Lane);
    void setFarLines(int, int);

    void deleteLeft();
    void deleteRight();

    int disBot();
    int disTop();

    void status();

};

void getTheta(Road &road);

#endif // ROAD_H
