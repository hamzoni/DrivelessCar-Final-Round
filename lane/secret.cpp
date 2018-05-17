//
// Created by nc on 11/05/2018.
//

#include "secret.h"

void processImgSe(cv::Mat &src){

    /*============================== pre-image-processing ======================*/
    cv::Mat roi_src, roi_hsv, roi_gray, mask_white, green;
    roi_src = src(imp::roi);
    cv::cvtColor(roi_src, roi_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(roi_src, roi_hsv, cv::COLOR_RGB2HSV);
    std::vector<std::vector<cv::Point>> hull;

    getMaskWhite(roi_gray, roi_hsv,mask_white,green, hull );

    /*============================== find lanes ======================*/
    // when take line from points, all of points will be considered so down_y = 0
    int STORE_DOWN_Y = conf::DOWN_Y;
    conf::DOWN_Y = 0;
    std::vector<Lane> lanes;

    findLaneFromImage(mask_white, lanes, conf::NUMLAYERS, conf::MIN_AREA_CNT,
                      conf::MIN_WIDTH_CNT,conf::MIN_POINTS, conf::MAX_DIS_TWO_POINTS);
    combineLanes(lanes, conf::MIN_POINTS_AFTER_COMBINE);

    std::vector<Lane> copy_lanes = lanes;
    Road road(conf::W_ROI, conf::H_ROI);
    road.turn_left = status::LEFT;
    ngaba(lanes, road, se::SLOPE_MAX_LINE_INJUNCTION, se::MIN_POINTS_AFTER_COMBINE, se::DIS_ROAD_INJUNCTION);
    status::NGABA = road.hasRoadInject;
    processLayerGridSe(road, hull , se::SLOPE_MAX_LINE_INJUNCTION, se::ANGLE_LINES_INJECTION, se::DIS_ROAD_INJUNCTION, false,
                       false);

    if(!road.hasRoadInject){
        conf::DOWN_Y = STORE_DOWN_Y;
        for(Lane &l : lanes){
            l.setLine(conf::H_ROI, conf::DOWN_Y);
        }
        /*============================== find left right ======================*/
        adaptiveLeftRight(lanes, road, 320, 320, 200, conf::MAX_ANGLE_TWO_LANES, conf::MIN_LEN_LANE, conf::RATIO_POINTS_LR);

        if(road.hasLeft && road.hasRight){
            road.fullLaneWhite = true;
            double slope_left = road.left.line_full.slope,
                    slope_right = road.right.line_full.slope;
            if(slope_left * slope_right > 0){
                if(cv::abs(slope_right) < 4 || cv::abs(slope_right) < 4){
                    if(status::LEFT){
                        road.deleteRight();
                    } else {
                        road.deleteLeft();
                    }
                }
            } else {
                int disLeft = distanceTwoPoints(road.pointBot, road.left.cnt[0]);
                int disRight = distanceTwoPoints(road.pointBot, road.right.cnt[0]);

                if(disLeft > disRight){
                    road.deleteLeft();
                } else {
                    road.deleteRight();
                }
            }

        } else {
            if(road.hasLeft && !status::LEFT){
                road.deleteLeft();
            } else if(road.hasRight && status::LEFT){
                road.deleteRight();
            }
            processLayerGridSe(road, hull , se::SLOPE_MAX_LINE_INJUNCTION, se::ANGLE_LINES_INJECTION, se::DIS_ROAD_INJUNCTION, false,
                               true);
        }

    } else {
        conf::DOWN_Y = conf::DOWN_Y_INJEC;
    }

    /*============================== process losing 2 lanes ======================*/
    if(!road.hasRight && !road.hasLeft){
        status::LOST_LANE = true;
        int dis = (int)(std::cos(pstatus::road.angle) * 100);
        if(pstatus::road.angle < 0){
            dis = -dis;
        }

        std::vector<cv::Point> left, right;
        left.push_back(pstatus::road.left.line.start);
        left.push_back(cv::Point(pstatus::road.left.line.end.x - dis, pstatus::road.left.line.end.y));

        road.setLeft(Lane(left, pstatus::road.H, conf::DOWN_Y));

        right.push_back(pstatus::road.right.line.start);
        right.push_back(cv::Point(pstatus::road.right.line.end.x - dis, pstatus::road.right.line.end.y));

        road.setRight(Lane(right, pstatus::road.H, conf::DOWN_Y));

        //road.pointTop = (road.left.line.end + road.right.line.end)/2;

    } else {
        status::LOST_LANE = false;
    }

    /*============================== calculate angles and speed======================*/

    conf::DOWN_Y = STORE_DOWN_Y;
    calculateAngleRoad(road);
    calculateAngle(road);
    //road.pointTop = (road.left.line.end + road.right.line.end)/2;

    getTheta(road);
    status::ANGLE = -road.angle;
    alluse(status::ANGLE);

    status::SPEED = (int)(fuzzy(status::STOP, -road.angleFar, status::NGABA, status::OBJ, status::LOST_LANE));


    if(angleTwoLines(pstatus::road.pointBot,pstatus::road.pointTop,
                     road.pointBot,road.pointTop) > 5){
        road.pointTop = (road.pointTop  +pstatus::road.pointTop)/2;
    }

    if(!status::LOST_LANE){
        pstatus::road = road;
    }

    /*============================== debug ======================*/
    if(conf::DEBUG){
        cv::Mat mask, gray3chan;
        mask = cv::Mat::zeros(conf::H_ROI, conf::W_ROI, CV_8UC3);
        cv::cvtColor(roi_gray, gray3chan, cv::COLOR_GRAY2BGR);
        draw(mask, gray3chan, mask_white, road, copy_lanes, status::obj_cors);
    }

    //reset config
    lanes.clear();
    copy_lanes.clear();
    conf::DOWN_Y = STORE_DOWN_Y;

}

