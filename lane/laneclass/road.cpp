#include "road.h"
#include "config.h"

Road::Road(){

}

Road::Road(int W, int H){
    this->pointBot.x = W/2;
    this->pointBot.y = H;

    this->pointTop.x = this->pointBot.x;
    this->pointTop.y = 0;

    this->hasLeft = false;
    this->hasRight = false;
    this->hasRoadInject = false;
    this->turn_left = false;
    this->fullLaneWhite = false;

    this->angle = 0;
    this->angleFar = 0;
    this->W = W;
    this->H = H;
}

void Road::setRight(Lane lane){
    this->right = lane;
    this->hasRight = true;
}

void Road::setLeft(Lane lane){
    this->left = lane;
    this->hasLeft = true;
}

void Road::setMiddle(Lane lane) {
    this->middle = lane;
    this->hasMiddle = true;
}

void Road::deleteLeft() {
    this->hasLeft = false;
    this->left.cnt.clear();
}

void Road::deleteRight(){
    this->hasRight = false;
    this->right.cnt.clear();
}

int Road::disBot() {
    return this->right.line_full.start.x - this->left.line_full.start.x;
}

int Road::disTop() {
    return this->right.line_full.end.x - this->left.line_full  .end.x;
}

void Road::setFarLines(int Y_BOT, int DOWN_Y_FAR) {
    this->left.setLineFar(Y_BOT, DOWN_Y_FAR);
    this->right.setLineFar(Y_BOT, DOWN_Y_FAR);
}

void Road::status() {
    std::cout << "dis bot: " << this->disBot() <<
              " - dis top: " << this->disTop() <<
              " - left: "<< this->hasLeft <<
              " - right: "<< this->hasRight<<
              " - inject: "<< this->hasRoadInject << std::endl;
}
double calculateAngle2Points(cv::Point &car, cv::Point &dst){

    if (dst.x == car.x){
        return 0;
    }
    if (dst.y == car.y){

        return dst.x < car.x ? -90 : 90;
    }

    double dx = dst.x - car.x;
    double dy = car.y - dst.y; // image coordinates system: car.y > dst.y

    if (dx < 0){
        return std::atan(-dx / dy) * 180 / PI;
    } else{
        return -std::atan(dx / dy) * 180 / PI;
    }
}

void getTheta(Road &road) {
    road.angle = calculateAngle2Points(road.pointBot, road.pointTop);
    road.angleFar = calculateAngle2Points(road.pointBotFar, road.pointTopFar);
}