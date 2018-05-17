#include "debug.h"
#include "lane.h"
#include "config.h"
#include "road.h"
#include "pointsprocess.h"

namespace debug{
    cv::Scalar white(255,255,255);
    cv::Scalar sky_blue(255,255,0);

    cv::Scalar green(0,255,0);
    cv::Scalar yellow(0,255,255);
    cv::Scalar red(0,0,255);
    cv::Scalar blue(206, 93, 22);
    cv::Scalar violet(255,0,255);
    cv::Scalar black(0,0,0);
}

void drawPoints(cv::Mat &src,
                std::vector<cv::Point> &point,
                int radius,
                cv::Scalar color) {
    for(cv::Point &p : point){
        cv::circle(src, p, radius, color , -1);
    }
}

void draw(cv::Mat           &mask,
          cv::Mat           &gray,
          cv::Mat           &bird,
          Road              &road,
          std::vector<Lane> lanes,
          std::vector<cv::Point> &obj_cors){

    cv::circle(mask, road.pointBot, conf::DIS_FAR_CAR, debug::green,1);
    cv::circle(mask, road.pointBot, conf::DIS_NEXT_CAR, debug::green,1 );
    cv::Point R;
    singPointBirdTo2d(road.pointTop, R, 1, 1);
    cv::circle(gray, R, 2, debug::green, -1);

    std::vector<cv::Point> point2d;
    pointBirdTo2d(road.left.cnt, point2d, 1, 1);
    pointBirdTo2d(road.right.cnt, point2d, 1, 1);
    drawPoints(gray, point2d, 2, debug::sky_blue);

    if(obj_cors.size() == 2){
        drawPoints(mask, obj_cors, 3, debug::red);
    }

    std::string s = "B " + std::to_string(road.disBot());
    cv::putText(mask, s, cv::Point2f(5,20), cv::FONT_HERSHEY_PLAIN, 1,  debug::sky_blue, 1);

    s = "T " + std::to_string(road.disTop());
    cv::putText(mask, s, cv::Point2f(5,45), cv::FONT_HERSHEY_PLAIN, 1,  debug::red, 1);

    s = "A " + std::to_string(-road.angle);
    cv::putText(mask, s, cv::Point2f(60,20), cv::FONT_HERSHEY_PLAIN, 1,  debug::sky_blue, 1);

    for(Lane &lane : lanes){
        drawPoints(mask, lane.cnt, 1, debug::yellow);
        cv::circle(mask, lane.line.start,2, debug::sky_blue,-1);
        cv::circle(mask, lane.line.end, 2, debug::sky_blue ,-1);
        cv::circle(mask, lane.line.anchor, 2,debug::white,-1);
        cv::line(mask, lane.line.start, lane.line.end, debug::yellow, 1);
    }

    if(road.hasRight){
        line(mask, road.right.line.start, road.right.line.end, debug::green,2);
        line(mask, road.right.line_far.start, road.right.line_far.end, debug::green,2);
        drawPoints(mask, road.right.cnt, 3, debug::red);
    }

    if(road.hasLeft){
        line(mask, road.left.line.start, road.left.line.end, debug::blue ,2);
        line(mask, road.left.line_far.start, road.left.line_far.end, debug::blue ,2);
        drawPoints(mask, road.left.cnt, 3, debug::green);
    }

    cv::line(mask, cv::Point((road.left.line.start.x + road.right.line.start.x)/2, road.H),
             cv::Point((road.left.line.end.x + road.right.line.end.x)/2, conf::DOWN_Y), debug::red , 1);
    cv::line(mask, road.pointBotFar, road.pointTopFar, debug::sky_blue , 2);
    cv::line(mask, road.pointBot, road.pointTop, debug::yellow , 2);

    cv::cvtColor(bird, bird, cv::COLOR_GRAY2BGR);
    cv::Mat mask2, mask3;
    cv::hconcat(mask, bird, mask2);
	cv::resize(gray, gray, cv::Size(gray.cols * bird.rows / gray.rows, bird.rows));
    cv::hconcat(mask2, gray, mask3);
    cv::imshow("Result", mask3);

}

void debugGreen(cv::Mat &mask,
                cv::Mat &mask_green,
                Road &road,
                std::vector<Lane> lanes){

    for(Lane &lane : lanes){
        drawPoints(mask, lane.cnt, 2, debug::red);
        cv::circle(mask, lane.line.start,2, debug::sky_blue,-1);
        cv::circle(mask, lane.line.end, 2, debug::sky_blue ,-1);
        cv::circle(mask, lane.line.anchor, 2,debug::white,-1);
        cv::line(mask, lane.line.start, lane.line.end, debug::yellow, 1);
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

    if(road.hasRoadInject){
        std::string s = road.turn_left ? "L" : "R";
        cv::putText(mask, s, cv::Point2f(5,20), cv::FONT_HERSHEY_DUPLEX, 1,  cv::Scalar(0,0,255,255), 1);
        if(road.hasLeft){
            for(cv::Point &p : road.left.cnt){
                cv::circle(mask, p, 2,cv::Scalar(0,255,0), -1);
            }
        } else {
            for(cv::Point &p : road.right.cnt){
                cv::circle(mask, p, 2,cv::Scalar(0,255,0), -1);
            }
        }
    }


    cv::line(mask, road.pointBot, road.pointTop, debug::yellow , 2);

    cv::Mat mask2, mask3;
    cv::cvtColor(mask_green, mask_green, cv::COLOR_GRAY2BGR);
    cv::hconcat(mask_green, mask , mask3);
    cv::imshow("road injection", mask3);

}

void drawParabol(cv::Mat                  &src,
                 std::vector<cv::Point>    &points,
                 std::vector<double>       &parabol){

    std::vector<cv::Point2f> list_point(src.cols);
    for(int x = 0; x < src.cols; x++){
        double xa = (double)x;
        double y = parabol[0] + parabol[1] * xa + parabol[2] * xa * xa;
        list_point[x].x = x;
        list_point[x].y = (int)y;
    }

    for(cv::Point &p : points){
        cv::circle(src, p, 3, cv::Scalar(255,255,255), -1);
    }

    for(int i = 1; i < list_point.size(); i++){
        cv::line(src,list_point[i-1],list_point[i], cv::Scalar(255,255,255));
    }

    list_point.clear();

}

