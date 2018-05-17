#ifndef IMAGEPROCESS_H
#define IMAGEPROCESS_H

#include "opencv2/highgui/highgui.hpp"
#include "layer.h"
#include "lane.h"

void separateLayers(cv::Mat             &src,
                    std::vector<Layer>  &layers,
                    int                 NUMLAYERS);

void birdView(cv::Mat &src,
              cv::Mat &dst,
              cv::Mat &matrixWrap);

void findCenterPoint(std::vector<Layer> &layers,
                     int                MIN_AREA_CNT,
                     int                MIN_WIDTH_CNT);

void findLaneFromImage(cv::Mat           &src,
                              std::vector<Lane> &lanes,
                              int               NUMLAYERS,
                              int               MIN_AREA_CNT,
                              int               MIN_WIDTH_CNT,
                              int               MIN_POINTS,
                              int               MAX_DIS_TWO_POINTS);

#endif // IMAGEPROCESS_H
