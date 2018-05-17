#include "laneimageprocess.h"
#include "config.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "pointsprocess.h"

//! Separate a image to small layers
void separateLayers(cv::Mat &src, std::vector<Layer> &layers, int NUMLAYERS){
    int H_LAYER = src.rows / NUMLAYERS;
    cv::Rect roi(0, 0, src.cols, H_LAYER);
    for(int i = 0; i < NUMLAYERS - 1; i++){
        roi.y = src.rows - (i+1)*H_LAYER;
        cv::Mat sub = src(roi);
        layers.push_back(Layer(sub, roi.x, roi.y));
    }

    roi.y = 0;
    roi.height = src.rows - H_LAYER*(NUMLAYERS - 1);
    cv::Mat sub = src(roi);
    layers.push_back(Layer(sub, roi.x, roi.y));
}


//! Bird View a image
void birdView(cv::Mat &src, cv::Mat &dst, cv::Mat &matrixWrap){
    cv::warpPerspective(src, dst, matrixWrap, cv::Size(conf::W_ROI, conf::H_ROI), cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);
}


void findCenterPoint(std::vector<Layer> &layers, int MIN_AREA_CNT, int MIN_WIDTH_CNT){
    std::vector<std::vector<cv::Point>> contours;

    for(unsigned int i = 0; i < layers.size(); i++){
        // find contours
        cv::findContours(layers[i].img, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

        // filter contours
        for(unsigned int i = 0; i < contours.size(); i++){
            if( (cv::contourArea(contours[i]) < MIN_AREA_CNT) ||
                cv::boundingRect(contours[i]).width  >= MIN_WIDTH_CNT){

                contours.erase(contours.begin() + i);
                i--;

            }
        }

        int cX, cY;
        cv::Moments M;

        for(std::vector<cv::Point> &cnt : contours){
            M = cv::moments(cnt);
            cX = (int)(M.m10 / M.m00);
            cY = (int)(M.m01 / M.m00);
            layers[i].points.push_back(cv::Point(cX, cY));
        }
     }
}

void findLaneFromImage(cv::Mat           &src,
                      std::vector<Lane> &lanes,
                      int               NUMLAYERS,
                      int               MIN_AREA_CNT,
                      int               MIN_WIDTH_CNT,
                      int               MIN_POINTS,
                      int               MAX_DIS_TWO_POINTS){
    std::vector<std::vector<cv::Point>> contours;
    std::vector<Layer> layers(NUMLAYERS);
    cv::Mat sub;
    int cX, cY;
    cv::Moments M;

    int H_LAYER = (int)(src.rows / NUMLAYERS);
    cv::Rect roi(0, 0, src.cols, H_LAYER);
    for(int i = 0; i < NUMLAYERS - 1; i++){
        roi.y = src.rows - (i+1)*H_LAYER;
        sub = src(roi);

        cv::findContours(sub, contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
        // filter contours
        for(unsigned int j = 0; j < contours.size(); j++){
            if( (cv::contourArea(contours[j]) < MIN_AREA_CNT) ||
                cv::boundingRect(contours[j]).width  >= MIN_WIDTH_CNT){

                contours.erase(contours.begin() + j);
                j--;
            }
        }

        for(std::vector<cv::Point> &cnt : contours){
            M = cv::moments(cnt);
            cX = (int)(M.m10 / M.m00) + roi.x;
            cY = (int)(M.m01 / M.m00) + roi.y;
            layers[i].points.push_back(cv::Point(cX, cY));
        }
    }

    pointsToLaneWithoutOriginAngle(layers, lanes, src.rows, MIN_POINTS, MAX_DIS_TWO_POINTS);

    layers.clear();
}