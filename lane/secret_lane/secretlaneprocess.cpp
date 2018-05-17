//
// Created by nc on 29/04/2018.
//

#include "secretlaneprocess.h"
#include "laneprocess.h"

void processContoursSe(std::vector<std::vector<cv::Point>> &contours){
    for (int i = 0; i < contours.size(); i++) {
        if(cv::contourArea(contours[i]) <= gre::MIN_AREA_GREEN_CNT){
            contours.erase(contours.begin() + i);
            i --;
        }
    }

    find_if_contours_cloes(contours);
}

void convex(std::vector<std::vector<cv::Point>> &contours){

    cv::Mat mask;
    mask = cv::Mat::zeros(cv::Size(gre::W_ROI, gre::H_ROI), CV_8UC3);
    int NUM  = 7, id;
    int H = conf::H_CUT / NUM;

    std::vector<std::vector<cv::Point>> new_cnt;

    for(auto &cnt : contours){
        std::vector<std::vector<cv::Point>> new_cnt_layers(NUM + 2);
        for(auto &p : cnt){
            id = p.y / H;
            new_cnt_layers[id].push_back(p);
        }
        for(auto &c : new_cnt_layers){
            if(c.size() > 0){
                std::vector<cv::Point> x;
                cv::convexHull(c, x, false );
                for(int j = 0; j < x.size(); j++){
                    x[j].x /= gre::X_RESIZE_RATIO;
                    x[j].y /= gre::Y_RESIZE_RATIO;
                }
                fillFullCirclePoints(x);
                new_cnt.push_back(x);
                drawPoints(mask, x, 2, debug::green);
            }
        }
        new_cnt_layers.clear();
    }

    contours = new_cnt;
    cv::imshow("test m", mask);

}

void removeNoisePointsLane( std::vector<Lane> &lanes){
    int old_size ;
    for(Lane &lane : lanes){
        old_size = lane.cnt.size();
        for(int i = 3; i < lane.cnt.size(); i++){
            double angle = angleThreePoints(lane.cnt[0], lane.cnt[i - 1], lane.cnt[i]);
            if(angle > 45){
                lane.cnt.erase(lane.cnt.begin() + i);
                i--;
            }
        }
        if(old_size > lane.cnt.size()){
            lane.setLane(0);
        }
    }
}

void processRoadGreenSe(cv::Mat &src, Road &road){

    std::vector<std::vector<cv::Point>> contours;
    findContoursNormColor(src, contours);
//    findContoursThresWhite(src, contours);
    processContoursSe(contours);

    std::vector<std::vector<cv::Point> > hull(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        cv::convexHull( cv::Mat(contours[i]), hull[i], false );

        for(int j = 0; j < hull[i].size(); j++){
            hull[i][j].x /= gre::X_RESIZE_RATIO;
            hull[i][j].y /= gre::Y_RESIZE_RATIO;
        }

        fillFullCirclePoints(hull[i]);
    }

    processLayerGridSe(road, hull , se::SLOPE_MAX_LINE_INJUNCTION, se::ANGLE_LINES_INJECTION, se::DIS_ROAD_INJUNCTION, false,
                       false);
}

void processLayerGridSe(Road &road,
                       std::vector<std::vector<cv::Point> > &contours,
                       double SLOPE_MAX_LINE_INJUNCTION,
                       int ANGLE_LINES_INJECTION,
                       int DIS_ROAD_INJUNCTION,
                       bool findRoadInj,
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
        pointsToLaneWithoutOriginAngle(layers, sub_lanes, tmp_road.H, gre::MIN_POINTS , gre::MAX_DIS_TWO_POINTS + 20 );

        if(findRoadInj && !tmp_road.hasRoadInject){
            tmp_road.hasRoadInject = roadInject(sub_lanes, tmp_road,
                                                SLOPE_MAX_LINE_INJUNCTION,
                                                ANGLE_LINES_INJECTION,
                                                DIS_ROAD_INJUNCTION);
        }

        if(findLanes && !tmp_road.hasRoadInject) {
//            removeNoiseLaneGreen(sub_lanes);
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

        for(Lane &l : total_lanes){
            l.setLine(tmp_road.H, tmp_road.H / 2);
        }
        separateLeftRightByDistancePointNoBird(total_lanes, tmp_road, 320, 320, tmp_road.H / 2);
        removeNoiseLRGreen(tmp_road);
        tranformRoad(tmp_road, road, true);
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
        debugGreen(mask, src, tmp_road, copy_total_lanes);
    }
}


//! Check if the road has two lanes but a lane is being wrong
/*!
 *
 * @param r
 * @param isBird    checking noise lane if being use birdview
 * @return          1 if left side is noise lane, 2 if right side, 0 if no noise lane
 */
int checkNoisLaneSe(Road &r, bool isBird){
    double ratio = (double)r.right.cnt.size() / (double)r.left.cnt.size();

    if(ratio < 1) ratio = 1/ ratio;

    if(ratio > se::RATIO_POINTS_LR){
        if(r.right.cnt.size() > r.left.cnt.size() ){
            return 1;
        }
        return 2;
    }

    if(distanceTwoPoints(r.left.cnt[0], r.left.cnt[r.left.cnt.size() - 1]) <= se::MIN_LEN_LANE) return 1;
    if(distanceTwoPoints(r.right.cnt[0], r.right.cnt[r.right.cnt.size() - 1]) <= se::MIN_LEN_LANE) return 2;

    cv::Point R;
    bool isCut= intersection(R, r.right.line, r.left.line);

    if(isCut && checkInRegion(R, r.W, r.H)){

        if(isBird || angleTwoLines(r.left.line_full,r.right.line_full) >= se::MAX_ANGLE_TWO_LANES
           || (R.y > r.left.cnt[0].y && R.y < r.left.cnt[r.left.cnt.size() - 1].y)
           || (R.y > r.right.cnt[0].y && R.y < r.right.cnt[r.right.cnt.size() - 1].y)){

            int disLeft = distanceTwoPoints(r.pointBot, r.left.cnt[0]);
            int disRight = distanceTwoPoints(r.pointBot, r.right.cnt[0]);

            if(disLeft > disRight){
                return 1;
            }

            return 2;
        }

        if(R.y >= r.left.cnt[0].y){
            return 1;
        } else if(R.y >= r.right.cnt[0].y){
            return 2;
        }

    }

    if( r.left.line.start.x >= r.right.line.start.x) {
        int disLeft = distanceTwoPoints(r.pointBot, r.left.cnt[0]);
        int disRight = distanceTwoPoints(r.pointBot, r.right.cnt[0]);

        if(disLeft > disRight){
            return 1;
        }

        return 2;
    }

    return 0;
}

//! Check noise lane and process if road has noise lane
/*!
 *
 * @param r
 * @param isBird    checking noise lane if being use birdview
 * @return          true if road has noise lane
 */
int removeNoiseLaneSe(Road &r, bool isBird){
    if(r.hasLeft && r.hasRight){
        int check = checkNoisLaneSe(r, isBird);
        if(check == 1){
            r.deleteLeft();
            return 1;
        } else if( check == 2){
            r.deleteRight();
            return 1;
        }
    }

    return 0;
}