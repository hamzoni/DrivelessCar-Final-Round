#include "line.h"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#define ESP_X 1

//! Init a line from two points with two Point type
Line::Line(){
    this->slope = 0.0000;
    this->angle = 0;
    this->c = 0;
    this->start = cv::Point();
    this->end = cv::Point();
    this->anchor = cv::Point();
}


Line::Line(double slope, cv::Point anchorPoint, int Y_BOT, int Y_TOP){
    this->anchor = anchorPoint;
    this->slope = slope;
    if(cv::abs(this->slope) > 572){
        this->slope = 572.5;
        this->angle = 90;
    } else {
        this->angle = std::atan(this->slope) * 180 / PI;
        if(this->angle < 0) this->angle += 180;
    }

    /*
    We want to find two points end of the line. Supose that F(x,y) is a point on the line,
    A(0, y_a) is start, B(WIDTH - 1, y_b) is end. What we need to do is find y_a and y_b.

    slope = (y_a - y)/(0 - x) = (y_b - y)/(WIDTH - x)
    slope = (hroi - ya)/(x - xa) = y/(xa - x)

    If you still confused, let visualize on graph.

    */
    int X_BOT = (int)((Y_BOT - anchorPoint.y)/this->slope + anchorPoint.x);
    int X_TOP = (int)(anchorPoint.x - (anchorPoint.y - Y_TOP)/this->slope);

    this->start = cv::Point(X_BOT, Y_BOT);
    this->end = cv::Point(X_TOP, Y_TOP);

    this->c = this->end.y - this->slope * this->end.x;
}

Line::Line(cv::Point p1, cv::Point p2){
    if(p1.y > p2.y){
        this->start = p1;
        this->end = p2;
    } else {
        this->start = p2;
        this->end = p1;
    }

    if(cv::abs(p1.x - p2.x) <= ESP_X){
        this->start.x += 1;
    }

    this->slope = (double)(this->start.y - this->end.y)/(this->start.x - this->end.x);

    if(cv::abs(this->slope) > 572){
        this->slope = 572.5;
        this->angle = 90;
    } else {
        this->angle = std::atan(this->slope) * 180 / PI;
        if(this->angle < 0) this->angle += 180;
    }

    this->c = this->end.y - this->slope * this->end.x;

}
