#include <greconf.h>
#include "laneprocess.h"
#include "lineprocess.h"
#include "pointsprocess.h"
#include "config.h"
#include "debug.h"
#include "status.h"

//! Check if two lanes can combine each other
/*!
 *
 * @param l1 the first lane
 * @param l2 the second lane
 * @return true if two lane can combine
 */
bool canCombine(Lane &l1, Lane &l2){
    double slope_1, slope_2, slope_two_points, angle_two_lines;
    cv::Point p_bot, p_top;

    p_bot = l1.line.anchor;
    p_top = l2.line.anchor;

    if(p_bot.y >= p_top.y){
        slope_1 = l1.line.slope;
        slope_2 = l2.line.slope;

        angle_two_lines = angleTwoLines(l1.line, l2.line);

        if (angle_two_lines <= conf::MAX_ANGLE_COMBINE_LANES){

            if(cv::abs(l1.line.end.x -l2.line.end.x)<= conf::MAX_X_DIS_COMBINE_LINES
               && cv::abs(l1.line.start.x -l2.line.start.x) <= conf::MAX_X_DIS_COMBINE_LINES) {
//                std::cout << 1 << std::endl;
                return 1;
            }

            slope_two_points = slopeTwoPoints(p_bot, p_top);
//            std::cout << slope_1 << " " <<slope_2 << "  "<< slope_two_points <<
//                      " " << p_bot << " " << p_top <<
//                      " " <<angle_two_lines << " " << angleTwoLines(slope_two_points, slope_1) << " " << angleTwoLines(slope_two_points, slope_2) << std::endl;

            if(cv::abs(slope_1) > cv::abs(slope_2)
//               && (slope_two_points - slope_1) * (slope_two_points - slope_2) < 0
               && angle_two_lines > angleTwoLines(slope_two_points, slope_1)
               && angle_two_lines > angleTwoLines(slope_two_points, slope_2)){
//                std::cout << 2 << std::endl;
                return 1;
            }
        }
    }
    return 0;
}

//! Try connect two or more lane which are breakdow
/*!
 *
 * @param lanes
 * @param MIN_POINTS_AFTER_COMBINE
 */
void combineLanes(std::vector<Lane> &lanes,
                  int               MIN_POINTS_AFTER_COMBINE){

    if(lanes.size() > 1){
        for(unsigned int i = 0; i < lanes.size() - 1; i++){
            bool combined = false;
            for (unsigned int j = i + 1; j < lanes.size(); j++) {

                if(canCombine(lanes[i], lanes[j])){

                    lanes[i].cnt.insert(lanes[i].cnt.end(), lanes[j].cnt.begin(), lanes[j].cnt.end());
                    combined = true;
                    lanes[i].setLane(0);
                    lanes.erase(lanes.begin() + j);
                    j--;
                }
            }

            if(combined){
                fillPoints(lanes[i].cnt, 0);
            }
        }
    }

    for(unsigned int i = 0; i < lanes.size(); i++){
        if(lanes[i].cnt.size() < MIN_POINTS_AFTER_COMBINE){
            lanes.erase(lanes.begin() + i);
            i--;
        }
    }
}


void fillPoints(std::vector<cv::Point> &cnt, int start){
    if(start < cnt.size() - 1){
        if (distanceTwoPoints(cnt[start], cnt[start + 1]) > conf::MAX_DIS_TWO_POINTS) {
            cnt.insert(cnt.begin() + start + 1,
                       cv::Point((cnt[start].x + cnt[start + 1].x) / 2, (cnt[start].y + cnt[start + 1].y) / 2));
            fillPoints(cnt, start);
        } else {
            fillPoints(cnt, ++start);
        }

    }
}




//! Find left - right side by distance points.
//! After that, take the most exactly lane and find again the other;
/*!
 *
 * @param lanes
 * @param road
 * @param MAX_DIS_LEFT  the max distance from the center of car to the left side
 * @param MAX_DIS_RIGHT the max distance from the center of car to the right side
 * @param MIN_Y the min y coordinate from the center of car to the lane
 */
void adaptiveLeftRight(std::vector<Lane> &lanes,
                       Road              &road,
                       int               MAX_DIS_LEFT,
                       int               MAX_DIS_RIGHT,
                       int               MIN_Y,
                       int MAX_ANGLE_TWO_LANES,
                       int MIN_LEN_LANE,
                       double RATIO_POINTS_LR){

    separateLeftRightByDistancePoint(lanes, road, MAX_DIS_LEFT, MAX_DIS_RIGHT, MIN_Y);
    if(removeNoiseLane(road, MAX_ANGLE_TWO_LANES, MIN_LEN_LANE, RATIO_POINTS_LR, true)){
        if(!road.hasLeft){
            findLeft(lanes,road, 640, MIN_Y);
        } else {
            findRight(lanes,road, 640, MIN_Y);
        }
    }

    if(road.left.cnt.size() > road.right.cnt.size()){
        findRight(lanes, road, MAX_DIS_RIGHT + MAX_DIS_LEFT, MIN_Y);
    } else {
        findLeft(lanes, road, MAX_DIS_LEFT + MAX_DIS_LEFT, MIN_Y);

    }
}


//! separte left side and right side by distance from center two to the lines
/*!
 * \param lanes     each lane consists cluster of points and a line fit the points
 * \param road
 */
void separateLeftRightByDistanceLine(std::vector<Lane>  &lanes,
                                     Road               &road){
    unsigned int num_lanes = lanes.size();
    int min_gap_left = road.W;
    int min_gap_right = min_gap_left;

    int index_left = -1;
    int index_right = -1;


    for(unsigned int i = 0; i < num_lanes; i++){
        if(lanes[i].cnt[0].y >= conf::DOWN_Y){
            int gap = road.pointBot.x - lanes[i].line.start.x;

            if(gap > 0 && gap < min_gap_left){
                min_gap_left = gap;
                index_left = i;
            }

            if(gap < 0 && cv::abs(gap) < min_gap_right){
                min_gap_right = cv::abs(gap);
                index_right = i;
            }
        }
    }

    if(index_left != -1){
        road.setLeft(lanes[index_left]);
    }
    if(index_right != -1){
        road.setRight(lanes[index_right]);
    }
}

//! Separate left side and right side by distance from center two to the lines
//! If two or more lanes are on the same side, find the nearest lane
/*!
 *
 * @param lanes
 * @param road
 * @param MAX_DIS_LEFT
 * @param MAX_DIS_RIGHT
 * @param MIN_Y
 */
void separateLeftRightByDistancePoint(std::vector<Lane> &lanes,
                                      Road              &road,
                                      int               MAX_DIS_LEFT,
                                      int               MAX_DIS_RIGHT,
                                      int               MIN_Y){
    int X_CENTER = road.pointBot.x;
    int index_left = -1;
    int index_right = -1;

    for(unsigned int i = 0; i < lanes.size(); i++){
        if(lanes[i].cnt[0].y >= MIN_Y) {
            int dis = distanceTwoPoints(road.pointBot, lanes[i].cnt[0]);
            int X = lanes[i].line.start.x;

            if (X < X_CENTER && dis < MAX_DIS_LEFT) {
                MAX_DIS_LEFT = dis;
                index_left = i;
            } else if (X > X_CENTER && dis < MAX_DIS_RIGHT) {
                MAX_DIS_RIGHT = dis;
                index_right = i;
            }
        } else {
            lanes.erase(lanes.begin() + i);
            i--;
        }
    }

    if(index_left != -1){
        road.setLeft(lanes[index_left]);
        lanes.erase(lanes.begin() + index_left);
        if(index_left < index_right) index_right--;
    }

    if(index_right != -1){
        road.setRight(lanes[index_right]);
        lanes.erase(lanes.begin() + index_right);

    }


}

//! Find left - right side by distance point when we don't use birdview
/*!
 *
 * @param lanes
 * @param road
 * @param MAX_DIS_LEFT
 * @param MAX_DIS_RIGHT
 * @param MIN_Y
 */
void separateLeftRightByDistancePointNoBird(std::vector<Lane>   &lanes,
                                            Road                &road,
                                            int                 MAX_DIS_LEFT,
                                            int                 MAX_DIS_RIGHT,
                                            int                 MIN_Y){
    int X_CENTER = road.pointBot.x;
    int index_left = -1;
    int index_right = -1;

    for(unsigned int i = 0; i < lanes.size(); i++){
        if(lanes[i].cnt[0].y >= MIN_Y) {
            int dis = distanceTwoPoints(road.pointBot, lanes[i].cnt[0]);
            int X = lanes[i].line.start.x;

            if (X < X_CENTER && dis < MAX_DIS_LEFT) {
                MAX_DIS_LEFT = dis;
                index_left = i;
            } else if (X > X_CENTER && dis < MAX_DIS_RIGHT) {
                MAX_DIS_RIGHT = dis;
                index_right = i;
            }
        }
    }
    if(index_left != -1){
        road.setLeft(lanes[index_left]);
    }

    if(index_right != -1){
        road.setRight(lanes[index_right]);
    }
//    removeNoiseLane(road, conf::MAX_DIS_TWO_POINTS, conf::MIN_LEN_LANE, conf::RATIO_POINTS_LR);
}

//! Find the left side if the road has right side.
/*!
 *
 * @param lanes
 * @param road
 * @param MAX_DIS_LEFT
 */
void findLeft(std::vector<Lane> &lanes,
              Road              &road,
              int               MAX_DIS_LEFT, int MIN_Y){

    unsigned int num_lanes = lanes.size();
    int X_CENTER = road.pointBot.x;
    int index_left = -1;
    int dis, disTop, disBot, Y_TOP_RIGHT = road.right.line_full.getX(MIN_Y);

    if(road.hasRight && road.hasLeft){
        disTop = Y_TOP_RIGHT - road.left.line_full.getX(MIN_Y);
        disBot = road.right.line_full.start.x - road.left.line_full.start.x;
        MAX_DIS_LEFT = disTop + disBot;
    }

    for(int i = 0; i < num_lanes; i++){
        if(lanes[i].line.start.x < X_CENTER &&
           lanes[i].line.anchor.y >= MIN_Y &&
           lanes[i].cnt.size() > road.left.cnt.size()){

            disTop = Y_TOP_RIGHT - lanes[i].line_full.getX(MIN_Y);
            disBot = road.right.line_full.start.x - lanes[i].line_full.start.x;

            if(disBot > 0 && disTop > 0){
                dis = disTop + disBot;

                if(dis < MAX_DIS_LEFT){
                    index_left = i;
                    MAX_DIS_LEFT = dis;
                }

            }
        }

    }

    if(index_left != -1){
        road.setLeft(lanes[index_left]);
    }
}

//! Find the right side if the road has the left side
/*!
 *
 * @param lanes
 * @param road
 * @param MAX_DIS_RIGHT
 */
void findRight(std::vector<Lane>    &lanes,
               Road                 &road,
               int                  MAX_DIS_RIGHT, int MIN_Y){

    unsigned int num_lanes = lanes.size();
    int X_CENTER = road.pointBot.x;
    int index_right = -1;
    int dis, disTop, disBot;
    if(road.hasRight && road.hasLeft){
        disTop = road.right.line_full.end.x - road.left.line_full.end.x;
        disBot = road.right.line_full.start.x - road.left.line_full.start.x;
        MAX_DIS_RIGHT = disTop + disBot;
    }


    for(unsigned int i = 0; i < num_lanes; i++){
        if(lanes[i].line.start.x > X_CENTER &&
           lanes[i].line.anchor.y >= MIN_Y &&
           lanes[i].cnt.size() > road.right.cnt.size()){

            disTop = lanes[i].line_full.end.x - road.left.line_full.end.x;
            disBot = lanes[i].line_full.start.x - road.left.line_full.start.x;

            if(disBot > 0 && disTop > 0){
                dis = disTop + disBot;

                if(dis < MAX_DIS_RIGHT){
                    index_right = i;
                    MAX_DIS_RIGHT = dis;
                }
            }
        }
    }

    if(index_right != -1){
        road.setRight(lanes[index_right]);
    }

}

//! Find the y coordinate to fit a suitable line to the lane
//! After that, generate lane(s) for road
/*!
 *
 * @param road
 */
void calculateAngleRoad(Road &road){

    int min_y = 0;
    if(road.hasLeft){
        min_y = road.left.cnt[conf::MIN_POINTS_AFTER_COMBINE - 1].y;
    }
    if(road.hasRight){
        if(min_y > road.right.cnt[conf::MIN_POINTS_AFTER_COMBINE - 1].y){
            min_y = road.right.cnt[conf::MIN_POINTS_AFTER_COMBINE - 1].y;
        }
    }

    if(min_y < conf::DOWN_Y){
        conf::DOWN_Y = conf::DOWN_Y_LOST_LANE;
    }

    if(road.hasLeft){
        road.left.setLine(road.pointBot.y, conf::DOWN_Y);
    }

    if(road.hasRight){
        road.right.setLine(road.pointBot.y, conf::DOWN_Y);
    }

    genLine(road);
}

//! Generate the losting line
//! calculate the top point of the car
/*!
 *
 * @param road
 */
void genLine(Road &road){
    int X_CENTER = road.pointBot.x;
    int DIS_BOT, DIS_TOP;
    DIS_BOT = conf::DIS_GEN_LINE_BOT;
    DIS_TOP = conf::DIS_GEN_LINE_TOP;
    if(status::BOTTLE){
        DIS_BOT += 30;
        DIS_TOP += 30;
    }


    if(!road.hasRight || !road.hasLeft){
        if(road.hasLeft){
            Line *line_left = &road.left.line;
            int DIS = X_CENTER - line_left->start.x;
            road.pointTop.x = line_left->end.x + DIS;

            std::vector<cv::Point> cnt;
            int x= road.left.line.start.x + DIS_TOP;
            if(x < road.pointBot.x){
                x = road.pointBot.x;
            }


            if(DIS > conf::MAX_DIS_BOT){
                cnt.push_back(cv::Point(conf::W_ROI,0));
                cnt.push_back(cv::Point(conf::W_ROI, conf::H_ROI));
            } else {
                cnt.push_back(cv::Point(x, conf::H_ROI));
                cnt.push_back(cv::Point(road.left.line.end.x + DIS_TOP, conf::DOWN_Y));
            }
            
            road.setRight(Lane(cnt, road.H, conf::DOWN_Y));
        } else if(road.hasRight){

            Line *line_right = &road.right.line;
            int DIS = line_right->start.x - X_CENTER;


            road.pointTop.x = line_right->end.x - DIS;

            std::vector<cv::Point> cnt;

            int x = road.right.line.start.x - DIS_TOP;
            if(x > road.pointBot.x){
                x = road.pointBot.x;
            }


            if(DIS > conf::MAX_DIS_BOT){
                cnt.push_back(cv::Point(0, 0));
                cnt.push_back(cv::Point(0, conf::H_ROI));
            } else {
                cnt.push_back(cv::Point(x, conf::H_ROI));
                cnt.push_back(cv::Point(road.right.line.end.x - DIS_TOP, conf::DOWN_Y));
            }



            road.setLeft(Lane(cnt,road.H, conf::DOWN_Y));

        } else {

            std::vector<cv::Point> cntLeft, cntRight;
            cntRight.push_back(cv::Point(X_CENTER + conf::DIS_GEN_LINE_BOT/2, conf::H_ROI));
            cntRight.push_back(cv::Point(road.pointTop.x + conf::DIS_GEN_LINE_TOP/2, conf::DOWN_Y));

            road.setRight(Lane(cntRight, road.H, conf::DOWN_Y));

            cntLeft.push_back(cv::Point(X_CENTER - conf::DIS_GEN_LINE_BOT/2, conf::H_ROI));
            cntLeft.push_back(cv::Point(road.pointTop.x - conf::DIS_GEN_LINE_TOP/2, conf::DOWN_Y));

            road.setLeft(Lane(cntLeft, road.H, conf::DOWN_Y));
        }

    }


}

void calculateFarRoad(Road &road, int Y_BOT, int Y_TOP){

    road.pointBotFar.y = Y_BOT;
    road.pointBotFar.x = (road.pointTop.x - road.pointBot.x)*(Y_BOT - road.pointTop.y)/(road.pointTop.y - road.pointBot.y) + road.pointTop.x;

}


//! Calculate direction of the road if object appear
/*!
 *
 * @param road
 * @param obj_cors      the two bottom points of box's object
 * @return              true if the object on the road
 */
bool processObject(Road &road, std::vector<cv::Point> obj_cors){
    cv::Point middle_obj( (obj_cors[0].x + obj_cors[1].x) /2 , (obj_cors[0].y + obj_cors[1].y)/2);
    int dis_car_obj = distanceTwoPoints(road.pointBot, middle_obj);

    // Check if object in processing area
    if(dis_car_obj >= conf::MIN_DIS_CAR_OBJ && dis_car_obj <= conf::MAX_DIS_CAR_OBJ){

        cv::Point *most_right, *most_left;
        if(obj_cors[0].x < obj_cors[1].x){
            most_left = &obj_cors[0];
            most_right = &obj_cors[1];
        } else {
            most_left = &obj_cors[1];
            most_right = &obj_cors[0];
        }

        Line obj(obj_cors[0],obj_cors[1]);
        cv::Point R_l, R_r;
        bool isCut_l = intersection(R_l, obj, road.left.line);
        bool isCut_r = intersection(R_r, obj, road.right.line);

        if(isCut_l && isCut_r){


            int b = R_l.x - most_right->x;
            int c = R_r.x - most_left->x;

            // Check if object on the road
            if(b < -10 && c > 10){

                // Object on the road but the two lane are too far from each other
                if(road.disBot() >= conf::MAX_DIS_BOT){

                    if(distanceTwoPoints(road.left.cnt[0], road.pointBot) <
                       distanceTwoPoints(road.right.cnt[0], road.pointBot)) {
                        std::cout << "1 ---------------- " << std::endl;
                        int dis = most_right->x -R_l.x ;
                        road.deleteLeft();
                        std::vector<cv::Point> cnt;
                        cnt.push_back(cv::Point(road.left.line_full .start.x + dis, conf::H_ROI));
                        cnt.push_back(*most_right);

                    } else {
                        road.pointTop = (*most_right + R_r ) / 2;
                        int dis = R_r.x - most_left->x;
                        road.deleteRight();

                        std::vector<cv::Point> cnt;
                        cnt.push_back(cv::Point(road.right.line_full.start.x - dis, conf::H_ROI));
                        cnt.push_back(*most_left);

                        road.setRight(Lane(cnt,road.H, conf::DOWN_Y));

                    }
                    return true;
                }

                // Object is too cloes to the car
                if(dis_car_obj <= conf::CLOSE_PROXIMITY_CAR_OBJ){
                    int center = (obj_cors[0].x + obj_cors[1].x)/2;
                    if(center < road.pointBot.x){
                        road.pointTop.x = most_right->x + conf::DIS_TURN_OBJ;
                        road.pointTop.y = most_right->y;
                        return true;
                    }
                    if(center > road.pointBot.x){
                        road.pointTop.x = most_left->x - conf::DIS_TURN_OBJ;
                        road.pointTop.y = most_left->y;
                        return true;
                    }
                }


                int a = R_l.x - most_left->x;
                int d = R_r.x - most_right->x;

                // Obj cut line right
                if(a >= 0 && d > 0){
                    road.pointTop = (*most_right + R_r)/2;
                    int dis = most_right->x -R_l.x ;
                    road.deleteLeft();
                    std::vector<cv::Point> cnt;
                    cnt.push_back(cv::Point(road.left.line_full .start.x + dis, conf::H_ROI));
                    cnt.push_back(*most_right);

                    road.setLeft(Lane(cnt,road.H, conf::DOWN_Y));
                }

                // Obj cut line left
                if(a < 0 && d <= 0){
                    road.pointTop = (*most_left + R_l)/2;
                    int dis = R_r.x - most_left->x;
                    road.deleteRight();

                    std::vector<cv::Point> cnt;
                    cnt.push_back(cv::Point(road.right.line_full.start.x - dis, conf::H_ROI));
                    cnt.push_back(*most_left);

                    road.setRight(Lane(cnt,road.H, conf::DOWN_Y));
                }

                // Obj on the roadway
                if(a < 0 && d > 0){
                    int dis_l = distanceTwoPoints(R_l, *most_left);
                    int dis_r = distanceTwoPoints(R_r, *most_right );
                    std::cout << " L " << dis_l<< " "<< dis_r<< std::endl;

                    if(dis_l > dis_r){

                        int dis = R_r.x - most_left->x;
                        road.deleteRight();
                        std::vector<cv::Point> cnt;
                        cnt.push_back(cv::Point(road.right.line_full.start.x - dis, conf::H_ROI));
                        cnt.push_back(*most_left);
                        road.setRight(Lane(cnt,road.H, conf::DOWN_Y));



                    } else {
                        int dis = most_right->x -R_l.x ;
                        road.deleteLeft();
                        std::vector<cv::Point> cnt;
                        cnt.push_back(cv::Point(road.left.line_full .start.x + dis, conf::H_ROI));
                        cnt.push_back(*most_right);
                        road.setLeft(Lane(cnt,road.H, conf::DOWN_Y));
                    }
                }
                return true;
            }
        }
    }

    return false;

}

//! Check if the road has two lanes but a lane is being wrong
/*!
 *
 * @param r
 * @param isBird    checking noise lane if being use birdview
 * @return          1 if left side is noise lane, 2 if right side, 0 if no noise lane
 */
int checkNoisLane(Road &r,
                  int MAX_ANGLE_TWO_LANES,
                  int MIN_LEN_LANE,
                  double RATIO_POINTS_LR,
                  bool isBird){
    double ratio = (double)r.right.cnt.size() / (double)r.left.cnt.size();
    if(ratio < 1) ratio = 1/ ratio;
    if(ratio > RATIO_POINTS_LR){
        if(r.right.cnt.size() > r.left.cnt.size() ){
            return 1;
        }
        return 2;
    }

    if(distanceTwoPoints(r.left.cnt[0], r.left.cnt[r.left.cnt.size() - 1]) <= MIN_LEN_LANE) return 1;

    if(distanceTwoPoints(r.right.cnt[0], r.right.cnt[r.right.cnt.size() - 1]) <= MIN_LEN_LANE) return 2;

    cv::Point R;
    bool isCut= intersection(R, r.right.line_full, r.left.line_full);

    if(isCut && checkInRegion(R, r.W, r.H)){

        if(isBird || angleTwoLines(r.left.line_full,r.right.line_full) >= MAX_ANGLE_TWO_LANES
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

    if( r.left.line_full.start.x >= r.right.line_full.start.x) {

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
int removeNoiseLane(Road &r,
                    int MAX_ANGLE_TWO_LANES,
                    int MIN_LEN_LANE,
                    double RATIO_POINTS_LR,
                    bool isBird){
    if(r.hasLeft && r.hasRight){
        int check = checkNoisLane(r, MAX_ANGLE_TWO_LANES, MIN_LEN_LANE, RATIO_POINTS_LR, isBird);
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

//! Check if two lane can match to a road injection
/*!
 *
 * @param l1
 * @param l2
 * @return
 */
bool isRoadInjection(Lane &l1, Lane &l2, int DIS_ROAD_INJUNCTION){
    cv::Point R;
    bool isCut= intersection(R, l1.line, l2.line);

    if(isCut && checkInRegion(R, conf::W_ROI, conf::H_ROI)){
        int dis = distanceTwoPoints(l1.cnt[0], R) + distanceTwoPoints(l2.cnt[0], R);
        if(dis < DIS_ROAD_INJUNCTION){
            return true;
        }
    }

    return false;
}

// process road injection
int ngaba(std::vector<Lane>    &lanes,
          Road                 &road,
          double SLOPE_MAX_LINE,
          int MIN_ANGLE_TWO_LINES,
          int DIS_ROAD_INJUNCTION){
    unsigned int num_lanes = lanes.size();

    if(num_lanes > 1){
        for(unsigned int i = 0; i < num_lanes-1; i++){

            double slope_i = lanes[i].line.slope;

            if(cv::abs(slope_i) <= SLOPE_MAX_LINE){

                for(unsigned int k = i + 1; k < num_lanes; k++){

                    double slope_k = lanes[k].line.slope;

                    if(cv::abs(slope_k) <= SLOPE_MAX_LINE
                       && slope_i * slope_k < 0
                       && lanes[i].cnt[lanes[i].cnt.size() - 1].y < lanes[k ].cnt[0].y
                       && angleTwoLines(lanes[i].line_full, lanes[k].line_full) <= MIN_ANGLE_TWO_LINES) {

                        if(isRoadInjection(lanes[i], lanes[k], DIS_ROAD_INJUNCTION)){
                            int index, current_x_bot, current_x_top;
                            if(road.turn_left){
                                index = lanes[i].line.end.x < lanes[k].line.end.x ? i : k ;
                                // the taked line is always right side if we want to turn left
                                road.setRight(lanes[index]);

                                // if right side is exists, we find left side
                                // x bottom point and x top point of left side are always less than
                                // x bottom ponnt and x top point of right side
                                current_x_bot = lanes[index].line.start.x;
                                current_x_top = (conf::MIN_X_INJUNCTION- lanes[index].line.c)/lanes[index].line.slope;

                                // current_x_top = lanes[index].line.end.x;

                                for(unsigned int j = 0; j < num_lanes; j++){
                                    if(j != index && lanes[j].line.start.x < current_x_bot
                                       && lanes[j].line.end.x < current_x_top){
                                        road.setLeft(lanes[j]);

                                        break;
                                    }
                                }

                            } else {
                                index = lanes[i].line.end.x > lanes[k].line.end.x ? i : k;
                                road.setLeft(lanes[index]);

                                //if left line is exists, we find right line
                                current_x_bot = lanes[index].line.start.x;
                                current_x_top = (conf::MIN_X_INJUNCTION - lanes[index].line.c)/lanes[index].line.slope;
                                // current_x_top = lanes[index].line.end.x;

                                for(unsigned int j = 0; j < num_lanes; j++){
                                    if(j != index && lanes[j].line.start.x > current_x_bot
                                       && lanes[j].line.end.x > current_x_top){
                                        road.setRight(lanes[j]);

                                        break;
                                    }
                                }

                            }
                            return 1;
                        }

                    } // end check loop k
                } // end loop k
            } // end check slope i
        } // end loop i
    } // end check size
    return 0;
}

void curveLane(std::vector<Lane> lanes){
    cv::Mat mask = cv::Mat::zeros(400, 320, CV_8UC3);

    for(Lane &lane : lanes){
        int size = (int)lane.cnt.size();
        cv::Point top, middle, bot, far;

        top = lane.cnt[size - 2];
        middle = lane.cnt[(int)(size / 2)];
        bot = lane.cnt[0];

        Line l(top, bot);

        far.x = (int)((middle.x + l.slope * (middle.y  - l.c)) / (l.slope * l.slope  +1) );
        far.y = l.getY(far.x);

        line(mask, bot, top, debug::yellow,1);
        line(mask, middle, far  , debug::yellow,1);
        drawPoints(mask, lane.cnt, 2, debug::green);
        cv::circle(mask,far, 3, debug::red,-1);
        cv::circle(mask, bot, 3, debug::violet,-1);
        cv::circle(mask, top, 3, debug::yellow,-1);
    }
    cv::imshow("aa", mask);

}