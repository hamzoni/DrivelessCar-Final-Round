//
// Created by nc on 06/05/2018.
//

#include "status.h"
#include "greconf.h"

namespace status{
    bool NGABA = false;
    bool LEFT = false;

    bool LOST_LANE = false;
    bool BOTTLE = false;
    bool OBJ = false;
    bool STOP = false;

    double ANGLE = 0;
    int SPEED = 0;
    int TRAFFIC = 1;
    std::vector<cv::Point> obj_cors;
}

namespace pstatus{
    Road road;
}


void get_closest_(cv::Vec3b &pixcel)
{
    if   (pixcel[1] < 50||pixcel[2] < 50)
    {
        pixcel = gre::_black;
        return;
    }

    if(pixcel[2]>255){
        pixcel = gre::_white;
    }
    int min_distance = 255;
    cv::Vec3b closest;
    for (auto color : gre::_colors_)
    {
        int distance = cv::abs(color[0] - pixcel[0]);

        if (distance < min_distance)
        {
            min_distance = distance;
//            closest = color;
            if (color[0] ==60)//select only red
            {
                closest = gre::_white ;
            }
            else
            {
                closest = gre::_black;
            }

        }
    }
    pixcel = closest;

}

void getMaskWhite(cv::Mat &gray,
                  cv::Mat &hsv,
                  cv::Mat &dst,
                  cv::Mat &green,
                  std::vector<std::vector<cv::Point>> &hull) {
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat thres, b;
    for (auto &kernel : gre::kernels_morp) {
        cv::morphologyEx(gray, gray, CV_MOP_CLOSE, kernel);
    }

    for (int y = 0; y < hsv.rows; ++y) {
        for (int x = 0; x < hsv.cols; ++x) {
            cv::Vec3b current = hsv.at<cv::Vec3b>(y, x);
            get_closest_(current);
            hsv.at<cv::Vec3b>(y, x) = current;
        }
    }

    cv::threshold(gray,thres,150,255,cv::THRESH_BINARY);
    cv::Mat hsv_channels[3];
    cv::split(hsv, hsv_channels);
    green = hsv_channels[2];
    cv::blur(thres,thres,cv::Size(5,5));
    cv::bitwise_and(green,thres,green);
    cv::threshold(green,green,50,255,cv::THRESH_BINARY);

    cv::threshold(gray, thres, conf::THRES, 255, cv::THRESH_BINARY);

    findContours(green, contours, cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE);
    processContours(contours);
    find_if_contours_cloes(contours);

    hull = std::vector<std::vector<cv::Point>>(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        cv::convexHull( cv::Mat(contours[i]), hull[i], false );
        cv::drawContours(thres, hull, i , debug::black, CV_FILLED);

        for(int j = 0; j < hull[i].size(); j++){
            hull[i][j].x /= gre::X_RESIZE_RATIO;
            hull[i][j].y /= gre::Y_RESIZE_RATIO;
        }
        fillFullCirclePoints(hull[i]);
    }

    birdView(thres,dst , imp::matrixWrap);
    findContours(dst, contours, cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE);

    double ratio;
    int i = 0;
    for(auto &cnt : contours){
        if(cv::contourArea(cnt) > conf::MIN_AREA_WHITE) {
            cv::RotatedRect r = cv::minAreaRect(cnt);
            cv::Rect ra = cv::boundingRect(cnt);
            ratio = (double) r.size.height / (double) r.size.width;
            if (ratio < 1) ratio = 1 / ratio;
            if (ratio <= 1.9) {
//                ratio = ra.area() / r.size.area();
                ratio = ra.area() /  cv::contourArea(cnt);

                if (ratio < 1) ratio = 1 / ratio;
                if (ratio <= 3) {
                    cv::drawContours(dst, contours, i, debug::black, CV_FILLED);
                }

            }
        } else {
            cv::drawContours(dst, contours, i, debug::black, CV_FILLED);
        }
        i++;
    }
}



void calculateAngle(Road &road){


    // calculate current angle
    cv::Point bot((road.left.line.start.x + road.right.line.start.x)/2, road.H);
    cv::Point top((road.left.line.end.x + road.right.line.end.x)/2, conf::DOWN_Y);

    Line middle( bot, top);
    if(!getPointFromDis(middle, road.pointBot, road.pointTop, conf::DIS_FAR_CAR)){
        road.pointTop.y = conf::DOWN_Y;
        road.pointTop.x= (road.left.line.end.x + road.right.line.end.x)/2;
    }


    // calculate next angle
    road.setFarLines(road.H, conf::DOWN_Y_FAR);

    bot.x = (road.left.line_far.start.x + road.right.line_far.start.x)/2;
    top.x = (road.left.line_far.end.x + road.right.line_far.end.x)/2;

    top.y = conf::DOWN_Y_FAR;
    middle = Line(bot, top);
    road.pointBotFar = road.pointTop;
    if(!getPointFromDis(middle, road.pointBotFar, road.pointTopFar, conf::DIS_NEXT_CAR)){
        road.pointTopFar.y = conf::DOWN_Y_FAR;
        road.pointTopFar.x= (road.left.line_far.end.x + road.right.line_far.end.x)/2;
    }

    int DIS = int(std::sin(angleTwoLines(road.pointBot,
                                         road.pointTop,
                                         road.pointBotFar,
                                         road.pointTopFar) * PI / 180) * conf::DIS_EXTERNAL);

    cv::Point add_point;
    middle = Line(road.pointBotFar, road.pointTopFar);

    if(getPointFromDis(middle, road.pointBotFar, add_point, DIS)){
        road.pointTop = add_point;
    }


    if(angleTwoLines(pstatus::road.pointBot,pstatus::road.pointTop,
                     road.pointBot,road.pointTop) > 5){
        road.pointTop = (road.pointTop  +pstatus::road.pointTop)/2;
    }

}

void updateObj(cv::Rect &box_obj, int X_RATIO, int Y_RATIO) {
    status::obj_cors.clear();
    if(status::OBJ) {
        std::vector<cv::Point> tmp;
        tmp.push_back(cv::Point(box_obj.x, box_obj.y + box_obj.height));
        tmp.push_back(cv::Point(box_obj.x + box_obj.width, box_obj.y + box_obj.height));
        point2d2Bird(tmp, status::obj_cors, X_RATIO, Y_RATIO, -imp::roi.x, -imp::roi.y);
    }
}

void showStatus() {
    std::cout << "turn left: " << status::LEFT << " - " <<
              "road inj: " << status::NGABA << " - " <<
              "bottle: " << status::BOTTLE << " - " <<
              "obj: " << status::OBJ << " - " <<
              "lost lane: " << status::LOST_LANE << " - " <<
              "angle: " << status::ANGLE << " - " <<
              "speed: " << status::SPEED << " - " <<
              std::endl;
}