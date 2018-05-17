#include "fuzzylogic.h"
#include "carconf.h"
#include "iostream"

double fuzzy(bool stop,
             double angle,
             bool NGABA,
             bool OBJ,
             bool LOST_LANE){

    if (stop || car::MAX_SPD == 0) return 0;
    if (LOST_LANE) return car::LOST_LANE_SPD;


    if(OBJ){
        return car::OBJ_SPD;
    }

    if(NGABA){
        return car::INJECT_SPD;
    }

    double speed;
    alluse(angle);


    angle = std::abs(angle);
    if(angle > 200) angle = 200;

    if(angle >= 0 && angle < car::BET){
       return car::MAX_SPD;
    }
    if(angle >= car::BET && angle < car::GAM){

        speed = car::MAX_SPD  - (angle - car::BET) * ( car::MAX_SPD - car::GAM_SPD) / (car::GAM - car::BET);
        return speed;
    }


    if( angle >= car::GAM){
        speed = car::GAM_SPD - (angle - car::GAM) * (car::GAM_SPD - car::DEL_SPD)/ (car::DEL - car::GAM);

        return speed;
    }

    return speed;
}

void alluse(double &angle) {
    double OA = refer::A - refer::O;
    double OB = refer::O - refer::B;
    if(angle < 0){
        angle = OB/std::abs(refer::b) * angle + refer::O;
    } else {
        angle = OA/refer::a * angle + refer::O;
    }
}
