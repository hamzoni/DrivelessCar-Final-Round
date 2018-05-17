#ifndef POINTSPROCESS_H
#define POINTSPROCESS_H

#include "layer.h"
#include "lane.h"

int distanceTwoPoints(cv::Point &p1,
                      cv::Point &p2);

double slopeTwoPoints(cv::Point &p1,
                      cv::Point &p2);

void findRelatedPoint(std::vector<Layer>        &layers,
                      std::vector<cv::Point>    &points,
                      unsigned int              current_layer,
                      int                       maxDistanceTwoPoints);

void pointsToLane(std::vector<Layer>    &layers,
                  std::vector<Lane>     &dst_lanes,
                  int                   H,
                  int                   minPoints,
                  int                   maxDistanceTwoPoints);

void pointsToLaneWithoutOrigin(std::vector<Layer>    &layers,
                               std::vector<Lane>     &dst_lanes,
                               int                   H,
                               int                   minPoints,
                               int                   minDistanceTwoPoints);

void pointsToLaneWithoutOriginAngle(std::vector<Layer>    &layers,
                                    std::vector<Lane>     &dst_lanes,
                                    int                   H,
                                    int                   minPoints,
                                    int                   minDistanceTwoPoints);

void groupPoints(std::vector<Layer> &layers,
                 int                MAX_GROUP_POINTS);

void point2d2Bird(std::vector<cv::Point>   &input,
                  std::vector<cv::Point>     &output,
                  double X_RATIO,
                  double Y_RATION,
                  int X_ROI,
                  int Y_ROI);

void point2d2Bird(std::vector<cv::Point>    &input,
                  std::vector<cv::Point>    &output,
                  double                       X_RATIO,
                  double                       Y_RATION);

void singPoint2d2Bird(cv::Point &point_src,
                      cv::Point &point_dst,
                      double    X_RATIO,
                      double    Y_RATION);

void singPointBirdTo2d(cv::Point &point_src,
                      cv::Point &point_dst,
                      double    X_RATIO,
                      double    Y_RATION);

void pointBirdTo2d(std::vector<cv::Point>   &input,
                   std::vector<cv::Point>    &output,
                   double                    X_RATIO,
                   double                    Y_RATION);


double angleTwoLines(double slope_1,
                     double slope_2);

double angleTwoLines(Line &l1,
                     Line &l2);

double angleTwoLines(cv::Point &l1_p1,
                     cv::Point &l1_p2,
                     cv::Point &l2_p1,
                     cv::Point &l2_p2);

double angleThreePoints(cv::Point &p1,
                        cv::Point &p2,
                        cv::Point &p3);


#endif // POINTSPROCESS_H
