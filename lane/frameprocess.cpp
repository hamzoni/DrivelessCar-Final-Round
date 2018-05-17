#include "frameprocess.h"

void processImg(cv::Mat &src){
    /*============================== pre-image-processing ======================*/
    cv::Mat roi_src, roi_hsv, roi_gray, mask_white, green;
    roi_src = src(imp::roi);
    cv::cvtColor(roi_src, roi_gray, cv::COLOR_RGB2GRAY);
    cv::cvtColor(roi_src, roi_hsv, cv::COLOR_RGB2HSV);
    std::vector<std::vector<cv::Point>> hull;
    getMaskWhite(roi_gray, roi_hsv,mask_white, green, hull);
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

    conf::DOWN_Y = STORE_DOWN_Y;

    for(Lane &l : lanes){
        l.setLine(conf::H_ROI, conf::DOWN_Y);
    }
    /*============================== find left right ======================*/
    adaptiveLeftRight(lanes, road, 320, 320, 100, conf::MAX_ANGLE_TWO_LANES, conf::MIN_LEN_LANE, conf::RATIO_POINTS_LR);
    if(road.hasLeft && road.hasRight){
        road.fullLaneWhite = true;
    }

    /*============================== detect green and road inject ======================*/

    processLayerGridA(road, hull, gre::SLOPE_MAX_LINE_INJUNCTION, gre::ANGLE_LINES_INJECTION, gre::DIS_ROAD_INJUNCTION, true);

    if(road.hasRoadInject) {
        status::NGABA = true;
        conf::DOWN_Y = conf::DOWN_Y_INJEC;
    } else {
        status::NGABA = false;
        conf::DOWN_Y = STORE_DOWN_Y;
    }

    /*============================== find bottle and remove extreme lane ======================*/
    if( road.hasLeft && road.hasRight){
        if( road.disBot() >= conf::MAX_DIS_BOT && !status::OBJ){
            if (distanceTwoPoints(road.pointBot, road.left.cnt[0]) >
                distanceTwoPoints(road.pointBot, road.right.cnt[0])) {
                road.deleteLeft();
            } else {
                road.deleteRight();
            }

            status::BOTTLE = true;
        } else {

            status::BOTTLE = false;
            if(road.fullLaneWhite && road.disTop() >= conf::DIS_TOP_INJ && road.disBot() <= conf::DIS_BOT_INJ){
                if(road.left.line_full.slope * road.right.line_full.slope < 0) {
                    if (road.turn_left) {
                        road.hasRight = false;
                    } else {
                        road.hasLeft = false;
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
            }
        }
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
        left.push_back(cv::Point(pstatus::road.left.line.end.x + dis, pstatus::road.left.line.end.y));

        road.setLeft(Lane(left, pstatus::road.H, conf::DOWN_Y));

        right.push_back(pstatus::road.right.line.start);
        right.push_back(cv::Point(pstatus::road.right.line.end.x + dis, pstatus::road.right.line.end.y));

        road.setRight(Lane(right, pstatus::road.H, conf::DOWN_Y));

    } else {
        status::LOST_LANE = false;
    }

    /*============================== process object ======================*/
    if(status::OBJ){
        if(status::obj_cors[0].y >= conf::DOWN_Y){
            conf::DOWN_Y = status::obj_cors[0].y;
        }
        calculateAngleRoad(road);
        status::OBJ = processObject(road, status::obj_cors);
    }

    /*============================== calculate angles and speed======================*/
    conf::DOWN_Y = STORE_DOWN_Y;
    calculateAngleRoad(road);
    calculateAngle(road);

    getTheta(road);
    status::ANGLE = -road.angle;
    alluse(status::ANGLE);
    status::SPEED = (int)(fuzzy(status::STOP, -road.angleFar, status::NGABA, status::OBJ, status::LOST_LANE));

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
//    showStatus();
}

