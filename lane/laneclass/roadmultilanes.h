//
// Created by nc on 16/04/2018.
//

#ifndef CDS_ROADMULTILANE_H
#define CDS_ROADMULTILANE_H

#include "road.h"

class RoadMulti{
public:
    int num_lanes;
    std::vector<Road> roads;
    RoadMulti();
};

#endif //CDS_ROADMULTILANE_H
