//
// Created by nc on 29/04/2018.
//

#include "greenlaneprocess.h"
#include "lineprocess.h"
#include "laneprocess.h"

bool canFill(cv::Point &p1, cv::Point &p2){
    int id_1 = p1.y / gre::H_LAYER;
    int id_2 = p2.y / gre::H_LAYER;

    if(id_1 != id_2) {

        bool related_1, related_2;


        related_1 = p1.x >= gre::roi_mask.x && p1.x <= gre::roi_mask.width;
        related_2 = p2.x >= gre::roi_mask.x && p2.x <= gre::roi_mask.width;

        if (related_1 || related_2) {
            related_1 = p1.y < gre::roi_mask.y && p2.y < gre::roi_mask.y;
            related_2 = p1.y > gre::roi_mask.height && p2.y > gre::roi_mask.height;

            if (!related_1 && !related_2) {
                if (distanceTwoPoints(p1, p2) > 10) {
                    return true;
                }
            }
        }
    }
    return false;

}

void fill(std::vector<cv::Point> &cnt, int &start, int end){

    if(start < end){
        int id_1 = cnt[start].y / gre::H_LAYER;
        int id_2 = cnt[start+1].y / gre::H_LAYER;

        if(distanceTwoPoints(cnt[start], cnt[start+1]) > 10 && id_1 != id_2){
            cnt.insert(cnt.begin() + start + 1, (cnt[start] + cnt[start + 1])/2);
            end++;

        } else {
            start++;
        }
        fill(cnt, start, end);
    }
}

void fillPointsGreen(std::vector<cv::Point> &cnt, int start){
    if(start < cnt.size() - 1){

        if(canFill(cnt[start], cnt[start  +1])){
            fill(cnt, start, start + 1);
            fillPointsGreen(cnt, start);
        } else {
            fillPointsGreen(cnt, start + 1);
        }
    }
}

void fillFullCirclePoints(std::vector<cv::Point> &cnt){
    fillPointsGreen(cnt, 0);
    int start = (int)cnt.size() - 1;

    if(canFill(cnt[start], cnt[0])){
        cnt.push_back(cnt[0]);
        fill(cnt, start, start + 1);
    }
}


bool isRoadInjectionGreen(Road &road,
                          Lane &l1,
                          Lane &l2,
                          int DIS_ROAD_INJUNCTION){
    cv::Point R;
    bool isCut= intersection(R, l1.line, l2.line);

    if(isCut
       && checkInRegion(R, conf::W_ROI, conf::H_ROI)
       && R.y > l1.line.anchor.y
       && R.y > l2.line.anchor.y) {
        int dis = distanceTwoPoints(l1.cnt[0], R) + distanceTwoPoints(l2.cnt[0], R);
        if(dis <= DIS_ROAD_INJUNCTION ){
            road.road_inject = l1.cnt[0];
            return true;
        }
    }

    return false;
}


// process road injection
bool roadInject(std::vector<Lane>    &lanes,
                Road                 &road,
                double SLOPE_MAX_LINE,
                int MIN_ANGLE_TWO_LINES,
                int DIS_ROAD_INJUNCTION) {

    int num_lanes = (int)lanes.size();

    if(num_lanes > 1){
        for(int i = num_lanes - 1; i >= 1; i--){
            double slope_i = lanes[i].line_full.slope;

            if(cv::abs(slope_i) <= SLOPE_MAX_LINE){
                for(int k = i - 1; k >= 0; k--){
                    double slope_k = lanes[k].line_full.slope;

//                    std::cout << slope_i << " " << slope_k << " " << angleTwoLines(slope_i, slope_k)  << std::endl;
                    if(cv::abs(slope_k) <= SLOPE_MAX_LINE
                       && slope_i * slope_k < 0
                       && lanes[i].cnt[0].y > lanes[k].cnt[lanes[k].cnt.size() - 1].y
                       && angleTwoLines(lanes[i].line_full, lanes[k].line_full) >= MIN_ANGLE_TWO_LINES) {

                        if(isRoadInjectionGreen(road, lanes[i], lanes[k], DIS_ROAD_INJUNCTION)){
                            int index;
                            if(road.turn_left){
                                index = lanes[i].line_full.end.x < lanes[k].line_full.end.x ? i : k ;
                                // the taked line is always right side if we want to turn left

                                road.setRight(lanes[index]);
                            } else {
                                std::cout << lanes[i].line_full << " R " << lanes[k].line_full;
                                index = lanes[i].line_full.end.x > lanes[k].line_full.end.x ? i : k;

                                road.setLeft(lanes[index]);

                            }
                            return true;
                        }

                    } // end check loop k
                } // end loop k
            } // end check slope i
        } // end loop i
    } // end check size
    return false;
}

//! After find a road injection, to ensure this road injection is being on the road,
//! we need check it is on the middle of the road
bool checkRoadInjectBeforeTranform(Road &src_road,
                                   Road &dst_road){
    cv::Point R = src_road.road_inject;
    cv::Point point_center_bot;

    singPoint2d2Bird(R, R, gre::X_RESIZE_RATIO, gre::Y_RESIZE_RATIO);



    // Get the point center of the road (the middle point of left and right)
    if(dst_road.hasLeft && dst_road.hasRight){
        point_center_bot.x = (dst_road.left.line.start.x + dst_road.right.line.start.x)/2;
    } else if (dst_road.hasLeft){
        point_center_bot.x = dst_road.left.line.start.x + conf::DIS_GEN_LINE_BOT/2;
    } else if (dst_road.hasRight){
        point_center_bot.x = dst_road.right.line.start.x - conf::DIS_GEN_LINE_BOT/2;
    }
    point_center_bot.y = dst_road.H;


    // Check if the point center and the point of road injection are on the same side of left side
    if(dst_road.hasLeft){
        double disTop = dst_road.left.line_full.getDisSide(point_center_bot);
        double disR = dst_road.left.line_full.getDisSide(R);

        std::cout << R<<" 1 " << disTop << " " << disR << " " << std::endl;

        if(disR * disTop < 0){
            src_road.hasRoadInject = false;
            return false;
        }
    }

    // Check if the point center and the point of road injection are on the same side of right side
    if(dst_road.hasRight){
        double disTop = dst_road.right.line_full.getDisSide(point_center_bot);
        double disR = dst_road.right.line_full.getDisSide(R);

        std::cout << R<<" 2 " << disTop << " " << disR << " " << std::endl;
        if(disR * disTop < 0){
            src_road.hasRoadInject = false;
            return false;
        }
    }

    if(distanceTwoPoints(R, dst_road.pointBot) > 200){
        src_road.hasRoadInject = false;
        return false;
    }

    return true;
}

//! The points are usually not on the shape,
//! "average" these points to reduce noise points;
void averagePointsLane(Lane &src_lane,
                       int  NUM_AVERAGE_POINTS){

    if(src_lane.cnt.size() >= NUM_AVERAGE_POINTS * 2){
        std::vector<cv::Point> cnt;
        int size = (int)src_lane.cnt.size() - NUM_AVERAGE_POINTS;
        cnt.push_back(src_lane.cnt[0]);
        for(int i = 0; i < size; i+= NUM_AVERAGE_POINTS){
            int x = 0, y = 0;
            for(int j = i; j < i + NUM_AVERAGE_POINTS; j++){
                x += src_lane.cnt[j].x;
                y += src_lane.cnt[j].y;
            }

            cnt.push_back(cv::Point(x/NUM_AVERAGE_POINTS, y/NUM_AVERAGE_POINTS));
        }

        src_lane.changePoints(cnt);
    }

}

//! The left and right lane may be wrong after finding
//! Check and remove the side which is not lane
/*!
 *
 * @param road
 */
void removeNoiseLRGreen(Road &road){
    if(road.hasLeft &&  road.hasRight){
        int disTop = road.right.line.end.x  - road.left.line.end.x;
        int disBot = road.right.line.start.x  - road.left.line.start.x;
        int dis = disBot - disTop;
        if(dis <= 20){
            if(cv::abs(road.left.line.slope) > cv::abs(road.right.line.slope)){
                road.hasLeft = false;
            } else {
                road.hasRight = false;
            }

        }
    }
}

//! A group of lanes may containing some lanes which are noise
//! We need keep main lanes
/*!
 *
 * @param lanes
 */
void removeNoiseLaneGreen(std::vector<Lane> &lanes){
    int size_lanes = (int)lanes.size();
    if(size_lanes > 1){
        int max_points = (int)lanes[0].cnt.size(), index = 0;
        for(int ii = 1; ii < size_lanes; ii++){
            if(lanes[ii].cnt.size() > max_points){
                max_points = (int)lanes[ii].cnt.size();
                index = ii;
            }
        }
        lanes.erase(lanes.begin(), lanes.begin() + index);
        lanes.erase(lanes.begin() + 1, lanes.end());
    }
}


//! Convert points of the current processing road to real road;
/*!
 *
 * @param src_road
 * @param dst_road
 * @param avarage_point  take number of group points to average point
 * @param roadInject
 */
void tranformRoad(Road &src_road,
                  Road &dst_road,
                  bool avarage_point,
                  bool roadInject){

    // Transform left points
    if(!dst_road.hasLeft && src_road.hasLeft){

        if(avarage_point){
            averagePointsLane(src_road.left, gre::NUM_AVERAGE_POINTS);
        }

        std::vector<cv::Point> cnt_left_int;
        point2d2Bird(src_road.left.cnt, cnt_left_int, gre::X_RESIZE_RATIO, gre::Y_RESIZE_RATIO);

        Lane tranLeft(cnt_left_int,dst_road.H, 0);
        // Check length of the begin point and the last point
        if(distanceTwoPoints(tranLeft.cnt[0], tranLeft.cnt[tranLeft.cnt.size() - 1]) >= conf::MIN_LEN_LANE){
            if(roadInject){
                dst_road.setLeft(tranLeft);
            } else if(dst_road.hasRight){
                int x_dst_road = dst_road.right.line_full.getX(100);
                int x_src_road = tranLeft.line_full.getX(100);
                if(x_src_road < x_dst_road &&
                   angleTwoLines(tranLeft.line, dst_road.right.line) <= conf::MAX_ANGLE_TWO_LANES){
                    dst_road.setLeft(tranLeft);
                }

            } else {
                dst_road.setLeft(tranLeft);

            }

        }

    }

    // Transform right points
    if(!dst_road.hasRight && src_road.hasRight){
        if(avarage_point){
            averagePointsLane(src_road.right, gre::NUM_AVERAGE_POINTS);
        }
        std::vector<cv::Point> cnt_right_int;
        point2d2Bird(src_road.right.cnt, cnt_right_int, gre::X_RESIZE_RATIO, gre::Y_RESIZE_RATIO);

        Lane tranRight(cnt_right_int, dst_road.H, 0);
        if(distanceTwoPoints(tranRight.cnt[0], tranRight.cnt[tranRight.cnt.size() - 1]) >= conf::MIN_LEN_LANE) {
            if(roadInject){
                dst_road.setRight(tranRight);
            } else if (dst_road.hasLeft) {
                int x_dst_road = dst_road.left.line_full.getX(100);
                int x_src_road = tranRight.line_full.getX(100);
                if (x_src_road > x_dst_road &&
                    angleTwoLines(tranRight.line, dst_road.left.line) <= conf::MAX_ANGLE_TWO_LANES) {
                    dst_road.setRight(tranRight);
                }
            } else {
                dst_road.setRight(tranRight);
            }
        }
    }

}

//! Find the left side if the road has right side.
/*!
 *
 * @param lanes
 * @param road
 * @param MAX_DIS_LEFT
 */
void findLeftGreen(std::vector<Lane> &lanes,
                   Road              &road,
                   int               MAX_DIS_LEFT){

    int num_lanes = (int)lanes.size();
    int X_CENTER = road.pointBot.x;
    int index_left = -1;
    int dis, disTop, disBot;
    if(road.hasRight && road.hasLeft){
        disTop = road.right.line_full.end.x - road.left.line_full.end.x;
        disBot = road.right.line_full.start.x - road.left.line_full.start.x;
        MAX_DIS_LEFT = disTop + disBot;
    }


    for(int i = 0; i < num_lanes; i++){
        if(lanes[i].line.start.x < X_CENTER && lanes[i].cnt.size() > road.left.cnt.size()){
            disTop = road.right.line_full.end.x - lanes[i].line_full.end.x;
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
void findRightGreen(std::vector<Lane>    &lanes,
                    Road                 &road,
                    int                  MAX_DIS_RIGHT){

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
        if(lanes[i].line.start.x > X_CENTER && lanes[i].cnt.size() > road.right.cnt.size()){
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

void lane2d2Bird(std::vector<Lane> &lanes,
                 double X_RATIO,
                 double Y_RATION){

    std::vector<cv::Point2f> point2f, tmp_out;

    for(int i = 0; i < lanes.size(); i++){
        point2f.clear();
        tmp_out.clear();
        for(cv::Point &p  : lanes[i].cnt){
            point2f.push_back(cv::Point(int(p.x * X_RATIO), int(p.y * Y_RATION)));
        }

        perspectiveTransform(point2f, tmp_out, imp::inverMatrixWrap);

        int size = (int)lanes[i].cnt.size();
        for(int j = 0; j < size; j++){
            lanes[i].cnt[j].x = (int)tmp_out[j].x;
            lanes[i].cnt[j].y = (int)tmp_out[j].y;
        }

        lanes[i].setLane(0);

    }
}

void processContours(std::vector<std::vector<cv::Point>> &contours){
    cv::Rect intersec;

    for (int i = 0; i < contours.size(); i++) {
        if(cv::contourArea(contours[i]) <= gre::MIN_AREA_GREEN_CNT){
            contours.erase(contours.begin() + i);
            i --;
        } else {
            cv::Rect box = cv::boundingRect(contours[i]);
            intersec =  box & gre::roi_obj;
            if(intersec.area() == box.area() || intersec.area() == 0){
                contours.erase(contours.begin() + i);
                i --;
            }
        }
    }
}


//! Check if two lanes can combine each other
/*!
 *
 * @param l1 the first lane
 * @param l2 the second lane
 * @return true if two lane can combine
 */
bool canCombineGreen(Lane &l1, Lane &l2){
    double angle_two_lines;
    cv::Point p_bot, p_top;

    p_bot = l1.line.anchor;
    p_top = l2.line.anchor;

    if(p_bot.y >= p_top.y){
        angle_two_lines = angleTwoLines(l1.line, l2.line);
        std::cout << "fad: " << angle_two_lines<< std::endl;

        if (angle_two_lines <= conf::MAX_ANGLE_COMBINE_LANES){
            if(cv::abs(l1.line.end.x -l2.line.end.x)<= conf::MAX_X_DIS_COMBINE_LINES
               && cv::abs(l1.line.start.x -l2.line.start.x) <= conf::MAX_X_DIS_COMBINE_LINES) {
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
void combineLanesGreen(std::vector<Lane> &lanes,
                  int               MIN_POINTS_AFTER_COMBINE){

    if(lanes.size() > 1){
        for(unsigned int i = 0; i < lanes.size() - 1; i++){

            for (unsigned int j = i + 1; j < lanes.size(); j++) {

                if(canCombineGreen(lanes[i], lanes[j])){
                    lanes[i].cnt.insert(lanes[i].cnt.end(), lanes[j].cnt.begin(), lanes[j].cnt.end());
                    lanes[i].setLane(0);
                    lanes.erase(lanes.begin() + j);
                    j--;
                }
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