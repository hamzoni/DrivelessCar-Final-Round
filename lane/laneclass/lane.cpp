#include "lane.h"
#include "config.h"
#define ESP_VX 0.001

Lane::Lane(){
    this->line = Line();
}
Lane::Lane(std::vector<cv::Point> contour, int H, int DOWN_Y){
    this->cnt = contour;

    this->H = H;
    this->setLane(DOWN_Y);

}

void Lane::setLane(int DOWN_Y){
    cv::Vec4f line;
    cv::fitLine(this->cnt, line, CV_DIST_L2, 0, 0.01, 0.01);

    if(line[0] < ESP_VX){

        line[0] = (float)ESP_VX;
    }
    this->line = Line(line[1]/line[0], cv::Point((int)line[2], (int)line[3]), this->H, DOWN_Y);
    this->line_full = this->line;
    this->line_far = this->line;

}
void Lane::changePoints(std::vector<cv::Point> points) {
    this->cnt = points;
    this->setLane(0);
}

void Lane::getLine(int Y_BOT, int Y_TOP, Line &dst_line, bool takePoints) {
    std::vector<cv::Point> new_cnt;
    for(cv::Point &p : this->cnt){
        if(p.y >= Y_TOP && p.y <= Y_BOT){
            new_cnt.push_back(p);
        } else {
            if(p.y < Y_TOP) break;
        }
    }

    if(takePoints) {
        if (new_cnt.size() < 3 && this->cnt.size() >= 3) {
            new_cnt.clear();
            new_cnt.push_back(this->cnt[0]);
            new_cnt.push_back(this->cnt[1]);
            new_cnt.push_back(this->cnt[2]);
        }
    }

    if(new_cnt.size() >= 3){
        cv::Vec4f line;
        cv::fitLine(new_cnt, line, CV_DIST_L2, 0, 0.01, 0.01);

        if(line[0] <= ESP_VX) line[0] = (float)ESP_VX;

        dst_line = Line( line[1]/line[0], cv::Point((int)line[2], (int)line[3]), Y_BOT, Y_TOP);

    } else {

        dst_line.end.y = Y_TOP;
        dst_line.end.x = (int)((Y_TOP - dst_line.c)/dst_line.slope);

        dst_line.start.y = Y_BOT;
        dst_line.start.x = (int)((Y_BOT - dst_line.c)/dst_line.slope);
    }
}

void Lane::setLine(int Y_BOT, int Y_TOP){
    this->getLine(Y_BOT, Y_TOP, this->line, true);
}

void Lane::setLineFar(int Y_BOT, int Y_TOP) {
    this->getLine(Y_BOT, Y_TOP, this->line_far, false);
}
