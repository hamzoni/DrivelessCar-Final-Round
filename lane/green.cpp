//
// Created by nc on 13/04/2018.
//

#include "green.h"
#include "debug.h"
#include "laneprocess.h"

void processRoadGreen(cv::Mat &src, Road &road){

    std::vector<std::vector<cv::Point>> contours;
//    findContoursNormColorHarCode(src, contours);
//    findContoursThres(src, contours);
    findContoursThresWhite(src, contours);
    processContours(contours);
    find_if_contours_cloes(contours);

    std::vector<std::vector<cv::Point> > hull(contours.size());

    for (int i = 0; i < contours.size(); i++) {
        cv::convexHull( cv::Mat(contours[i]), hull[i], false );
        for(int j = 0; j < hull[i].size(); j++){
            hull[i][j].x /= gre::X_RESIZE_RATIO;
            hull[i][j].y /= gre::Y_RESIZE_RATIO;
        }
        fillFullCirclePoints(hull[i]);
    }
    processLayerGridA(road, hull, gre::SLOPE_MAX_LINE_INJUNCTION, gre::ANGLE_LINES_INJECTION, gre::DIS_ROAD_INJUNCTION, false);
}


void processLayerGridA(Road &road,
                       std::vector<std::vector<cv::Point> > &contours,
                       double SLOPE_MAX_LINE_INJUNCTION,
                       int ANGLE_LINES_INJECTION,
                       int DIS_ROAD_INJUNCTION,
                       bool findLanes){

    Road tmp_road(gre::W_ROI , gre::H_ROI);
    tmp_road.turn_left = road.turn_left;
    std::vector<Lane> total_lanes;
    for(int i = 0; i < contours.size(); i++) {
        std::vector<Layer> layers(gre::NUMLAYERS);
        for (cv::Point &p : contours[i]) {
                int x_grid = (p.y / gre::H_LAYER);
                int y_grid = (p.x / gre::W_SUB_LAYER);

                gre::grid[x_grid][y_grid][0] += p.x;
                gre::grid[x_grid][y_grid][1] += p.y;
                gre::grid[x_grid][y_grid][2]++;

        }

        for (int ii = (int) gre::grid.size() - 1; ii >= 0; ii--) {
            int index_layer = (int) gre::grid.size() - ii - 1;
            unsigned int size_j = gre::grid[ii].size();
            for (unsigned int j = 0; j < size_j; j++) {

                if (gre::grid[ii][j][2] > 0) {
                    layers[index_layer].points.push_back(cv::Point(gre::grid[ii][j][0] / gre::grid[ii][j][2],
                                                                   gre::grid[ii][j][1] / gre::grid[ii][j][2]));

                    gre::grid[ii][j][0] = gre::grid[ii][j][1] = gre::grid[ii][j][2] = 0;

                    int x = -1, y, size;

                    for (int k = j + 1; k < gre::grid[ii].size(); k++) {
                        if (gre::grid[ii][k][2] > 0) {
                            x = gre::grid[ii][k][0];
                            y = gre::grid[ii][k][1];
                            size = gre::grid[ii][k][2];
                            gre::grid[ii][k][0] = gre::grid[ii][k][1] = gre::grid[ii][k][2] = 0;
                        }
                    }

                    if (x != -1) {
                        layers[index_layer].points.push_back(cv::Point(x / size, y / size));
                    }
                    break;
                }
            }
        }


        std::vector<Lane> sub_lanes;
        pointsToLaneWithoutOriginAngle(layers, sub_lanes, tmp_road.H, gre::MIN_POINTS , gre::MAX_DIS_TWO_POINTS + 10);
        if(!tmp_road.hasRoadInject){
            tmp_road.hasRoadInject = roadInject(sub_lanes, tmp_road,
                                            SLOPE_MAX_LINE_INJUNCTION,
                                            ANGLE_LINES_INJECTION,
                                            DIS_ROAD_INJUNCTION);
        }

        if(findLanes && !tmp_road.hasRoadInject) {
            removeNoiseLaneGreen(sub_lanes);
            total_lanes.insert(total_lanes.end(), sub_lanes.begin(), sub_lanes.end());
        }
        layers.clear();
    }

    std::vector<Lane> copy_total_lanes;
    if(gre::DEBUG){
        copy_total_lanes = total_lanes;
    }

    if(tmp_road.hasRoadInject){
        checkRoadInjectBeforeTranform(tmp_road, road);
        if(tmp_road.hasRoadInject) {
            int current_x_top;
            if (road.turn_left) {
                if (!road.hasLeft) {
                    current_x_top = tmp_road.right.line.getX(gre::DOWN_Y_INJUNCTION);

                    for (unsigned int j = 0; j < total_lanes.size(); j++) {
                        if (total_lanes[j].line.start.x < tmp_road.pointBot.x
                            && total_lanes[j].line.end.x < current_x_top) {
                            tmp_road.setLeft(total_lanes[j]);
                            break;
                        }
                    }
                }
                road.deleteRight();
            } else {
                if (!road.hasRight) {
                    current_x_top = tmp_road.left.line.getX(gre::DOWN_Y_INJUNCTION);

                    for (unsigned int j = 0; j < total_lanes.size(); j++) {
                        if (total_lanes[j].line.start.x > tmp_road.pointBot.x
                            && total_lanes[j].line.end.x > current_x_top) {
                            tmp_road.setRight(total_lanes[j]);
                            break;
                        }
                    }
                }
                road.deleteLeft();
            }
            road.hasRoadInject = true;
            tranformRoad(tmp_road, road, false, true);
        }
    }


    if(findLanes && !tmp_road.hasRoadInject){
        if(!road.hasRight && !road.hasLeft){
            for(Lane &l : total_lanes){
                l.setLine(tmp_road.H, tmp_road.H / 2);
            }
                    separateLeftRightByDistancePointNoBird(total_lanes, tmp_road, 320, 320, tmp_road.H / 2);
                    removeNoiseLRGreen(tmp_road);
                    tranformRoad(tmp_road, road, true);
        } else {
            lane2d2Bird(total_lanes, gre::X_RESIZE_RATIO, gre::Y_RESIZE_RATIO);
            for(Lane &lane: total_lanes){
                lane.setLine(conf::H_ROI, conf::DOWN_Y + 100);
            }

            if(!road.hasLeft){
                findLeft(total_lanes, road, 640, 100);
            }
            if(!road.hasRight){
                findRight(total_lanes, road, 640, 100);

            }

            removeNoiseLane(road, conf::MAX_ANGLE_TWO_LANES, conf::MIN_LEN_LANE, conf::RATIO_POINTS_LR, true);

        }
    }

    if(gre::DEBUG){
        cv::Mat mask;
        mask = cv::Mat::zeros(cv::Size(conf::W_ROI, conf::H_ROI), CV_8UC3);
        cv::drawContours(mask, contours, -1., cv::Scalar(255), 2);
        cv::Mat src  = cv::Mat::zeros(cv::Size(conf::W_ROI, conf::H_ROI), CV_8UC1);
        for(auto &cnt : contours){
            for(int i = 0; i < cnt.size(); i++){
                std::string s = std::to_string(i);
                cv::circle(mask, cnt[i],2, debug::sky_blue,-1);
            }
        }
        debugGreen(mask, src, tmp_road, total_lanes);
    }
}

void detectLaneGreen(cv::Mat &src, Road &road){
    cv::resize(src, src, cv::Size(gre::W_ROI, gre::H_ROI));
    // process layer
    std::vector<Layer> layers;
    separateLayers(src, layers, 30);
    findCenterPoint(layers, 2, 2000);
//    groupPoints(layers, 10);

    // process point
    std::vector<Lane> lanes;
    pointsToLane(layers, lanes, src.rows,3, 20);
//    combineLanes(lanes);

    for(Lane &l : lanes){
        l.setLine(src.rows, 100);
    }
    Road tmp_road(src.cols, src.rows);
    tmp_road.turn_left = road.turn_left;

    separateLeftRightByDistancePointNoBird(lanes, tmp_road, 320, 320, tmp_road.H / 2);

    road.hasRoadInject = roadInject(lanes, tmp_road,
                             gre::SLOPE_MAX_LINE_INJUNCTION,
                             gre::ANGLE_LINES_INJECTION,
                             gre::DIS_ROAD_INJUNCTION);

    tranformRoad(tmp_road, road, true, true);

    if(gre::DEBUG){
        cv::Mat mask;
        mask = cv::Mat::zeros(src.size(), CV_8UC3);
        debugGreen(mask, src, tmp_road, lanes);
    }

    //reset config
    layers.clear();
    lanes.clear();
}

void detectRoadInject(cv::Mat &src,
                      Road &road,
                      double SLOPE_MAX_LINE_INJUNCTION,
                      int ANGLE_LINES_INJECTION,
                      int DIS_ROAD_INJUNCTION){
    cv::Mat b;
    birdView(src, b , imp::matrixWrap);
    std::vector<Lane> lanes;
    findLaneFromImage(b, lanes, conf::NUMLAYERS, conf::MIN_AREA_CNT,
                      2000, 6, conf::MAX_DIS_TWO_POINTS);

    road.hasRoadInject = roadInject(lanes, road,
                                        SLOPE_MAX_LINE_INJUNCTION,
                                        ANGLE_LINES_INJECTION,
                                        DIS_ROAD_INJUNCTION);

    if(gre::DEBUG){
        cv::Mat mask = cv::Mat::zeros(b.size(), CV_8UC3);

        for(Lane &lane : lanes){
            for(cv::Point &p : lane.cnt){
                cv::circle(mask, p, 2, debug::sky_blue, -1);
            }
            cv::circle(mask, lane.line.start, 1,cv::Scalar(255,255,0),-1);
            cv::circle(mask, lane.line.end, 1,cv::Scalar(255,255,255),-1);
            line(mask, lane.line.start, lane.line.end, cv::Scalar(0,255,0),1);
            cv::circle(mask, lane.line.anchor, 1 ,cv::Scalar(255,0,255),-1);
        }

        if(road.hasRight){
            line(mask, road.right.line.start, road.right.line.end, debug::green,2);
            for(cv::Point &p : road.right.cnt){
                cv::circle(mask, p, 3, debug::red,-1);
            }

        }

        if(road.hasLeft){
            line(mask, road.left.line.start, road.left.line.end, debug::blue ,2);

            for(cv::Point &p : road.left.cnt){
                cv::circle(mask, p, 3, debug::green,-1);
            }
        }

//        cv::hconcat(mask, b, b);
        cv::imshow("Road inject b", b);
    }
}