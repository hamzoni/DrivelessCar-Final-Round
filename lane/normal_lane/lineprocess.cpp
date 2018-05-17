
#include "lineprocess.h"
#include "iostream"

//! Calculate intersection of two line
/*!
 * \param R     output inttersection point
 * \param l1
 * \param l2
 * \return      true if intersection point exist, false if two line are parallel
 */
bool intersection(cv::Point &R, Line &l1, Line &l2){
    double D = l1.slope - l2.slope;

    if(cv::abs(D) > 0.005){
        R.x = (l2.c - l1.c)/D;
        R.y = l1.slope*(double)R.x + l1.c;

        return true;

    } else {
        return false;
    }

}

bool checkInRegion(cv::Point &p, int W, int H){
    if(p.x >= 0 && p.x <= W && p.y >= 0 && p.y <= H){
        return true;
    }
    return false;
}





//! Find a point on the line which far from the anchor point by a distance
//! The function finds the most top point
/*!
 *
 * @param line
 * @param anchorPoint
 * @param src_point
 * @param DIS
 * @return              false if no point
 */
bool getPointFromDis(Line &line,
                     cv::Point &anchorPoint,
                     cv::Point &src_point,
                     int DIS) {
    double f = line.c - anchorPoint.y;
    double a = 1 + line.slope * line.slope;
    double b = 2*(-anchorPoint.x + line.slope*f);
    double c = f*f - DIS * DIS  + anchorPoint.x * anchorPoint.x;
    double delta = b*b - 4 * a * c, x1, x2, y1, y2;

    if(delta > 0){
        x1 = (-b - sqrt(delta))/(2 * a);
        x2 = (-b + sqrt(delta))/(2 * a);

        y1 = line.getY(x1);
        y2 = line.getY(x2);

        if((y1 - anchorPoint.y) * (y2 - anchorPoint.y) < 0){
            if(y1 > y2){
                x1 = x2;
                y1 = y2;
            }
            src_point.x = (int)x1;
            src_point.y = (int)y1;

            return true;
        }
    }

    return false;
}