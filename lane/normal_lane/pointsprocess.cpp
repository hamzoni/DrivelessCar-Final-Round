#include "pointsprocess.h"
#include "line.h"
#include "config.h"

int distanceTwoPoints(cv::Point &p1,
                      cv::Point &p2){
    return cv::abs(p1.x - p2.x) + cv::abs(p1.y - p2.y);
}

double slopeTwoPoints(cv::Point &p1,
                      cv::Point &p2){

    int slope_x = p2.x - p1.x;
    if( cv::abs(slope_x) < 0.01) return 10.;
    return (double)(p2.y - p1.y)/(double)slope_x;
}



double angleTwoLines(Line &l1,
                     Line &l2){
    return cv::abs(l1.angle - l2.angle);
}


double angleTwoLines(double slope_1, double slope_2){
    // NOTE : slope_1 and slope_2 are not zeros;
    double angle_1, angle_2;
    angle_1 = std::atan(slope_1) * 180 / PI;
    angle_2 = std::atan(slope_2) * 180 / PI;

    if(angle_1 < 0){
        angle_1 = 180 + angle_1;
    }
    if(angle_2 < 0){
        angle_2 = 180 + angle_2;
    }
    return cv::abs(angle_1 - angle_2);
}

double angleTwoLines(cv::Point &l1_p1,
                     cv::Point &l1_p2,
                     cv::Point &l2_p1,
                     cv::Point &l2_p2){

    return angleTwoLines(slopeTwoPoints(l1_p1, l1_p2), slopeTwoPoints(l2_p1, l2_p2));
}


double angleThreePoints(cv::Point &p1,
                        cv::Point &p2,
                        cv::Point &p3){
    double angle = angleTwoLines(slopeTwoPoints(p1, p2), slopeTwoPoints(p2, p3));

    if(p3.y > p2.y){
        return 180 - angle;
    }
    return angle;
}

//! This is a recursive function. It finds a point of current layer which is nearest with
//! current point of previous layer
void findRelatedPoint(std::vector<Layer> &layers,
                      std::vector<cv::Point> &points,
                      unsigned int current_layer,
                      int maxDistanceTwoPoints){
    unsigned int size_points = layers[current_layer].points.size();

    if(size_points > 0){
        int index = -1;
        int minDistance = maxDistanceTwoPoints;

        for(unsigned int i = 0; i < size_points; i++){
            int distance = distanceTwoPoints(layers[current_layer].points[i], points[points.size() - 1]);

            if(distance < minDistance){
                minDistance = distance;
                index = i;
            }
        }

        if(index != -1){
            points.push_back(layers[current_layer].points[index]);
            layers[current_layer].points.erase(layers[current_layer].points.begin() + index);
        }
    }

    if(current_layer < layers.size() - 1){
        findRelatedPoint(layers, points, ++current_layer, maxDistanceTwoPoints);
    }
}

//! This is a recursive function. It finds a point of current layer which is nearest with
//! current point of previous layer
void findRelatedPointAngle(std::vector<Layer> &layers,
                      std::vector<cv::Point> &points,
                      unsigned int current_layer,
                      int maxDistanceTwoPoints){
    unsigned int size_points = layers[current_layer].points.size();
    int index = -1;

    if(size_points > 0){
        int minDistance = maxDistanceTwoPoints;
        int size_points_lane = (int)points.size();
        for(unsigned int i = 0; i < size_points; i++){
            int distance = distanceTwoPoints(layers[current_layer].points[i], points[size_points_lane - 1]);
            if(distance < minDistance ){
                double angle = angleThreePoints(points[0], points[size_points_lane - 1], layers[current_layer].points[i]);
                if(angle < conf::MAX_ANGLE_TWO_POINTS){
                    minDistance = distance;
                    index = i;
                }
            }
        }
    }

    if(index != -1){
        points.push_back(layers[current_layer].points[index]);
        layers[current_layer].points.erase(layers[current_layer].points.begin() + index);



    }
    if(current_layer < layers.size() - 1){
        findRelatedPointAngle(layers, points, ++current_layer, maxDistanceTwoPoints);
    }

}

//! This is a recursive function. It finds a point of current layer which is nearest with
//! current point of previous layer
void findRelatedPointScan(std::vector<Layer> &layers,
                      std::vector<cv::Point> &points,
                      unsigned int current_layer,
                      int maxDistanceTwoPoints){
    int index = -1, indexlayer = current_layer;
    int distance;
    int minDistance = maxDistanceTwoPoints;

    for(int i = current_layer; i < current_layer + 2; i++){

        int size_points = (int)layers[i].points.size();

        if(size_points > 0){
            for(unsigned int j = 0; j < size_points; j++){
                distance = distanceTwoPoints(layers[i].points[j], points[points.size() - 1]);
                if(distance < minDistance ){
                    minDistance = distance;
                    index = j;
                    indexlayer = i;
                }
            }
        }
    }

    if(index != -1){
        points.push_back(layers[indexlayer].points[index]);
        layers[indexlayer].points.erase(layers[indexlayer].points.begin() + index);
    }

    if(current_layer < layers.size() - 3){
        current_layer += indexlayer - current_layer + 1;
        findRelatedPointScan (layers, points, current_layer, maxDistanceTwoPoints);
    }
}

//! This is a recursive function. It finds a point of current layer which is nearest with
//! current point of previous layer
void findMinPoint(std::vector<Layer> &layers,
                      std::vector<cv::Point> &points,
                      unsigned int current_layer,
                      int maxDistanceTwoPoints, int MIN_POINTS){

    unsigned int size_points = layers[current_layer].points.size();
    int index = -1;

    if(size_points > 0){
        int minDistance = maxDistanceTwoPoints;
        int size_points_lane = (int)points.size();

        for(unsigned int i = 0; i < size_points; i++){
            int distance = distanceTwoPoints(layers[current_layer].points[i], points[size_points_lane - 1]);

            if(distance < minDistance ){
                minDistance = distance;
                index = i;
            }
        }
    }

    if(index != -1){
        points.push_back(layers[current_layer].points[index]);
        layers[current_layer].points.erase(layers[current_layer].points.begin() + index);

        if(points.size() < MIN_POINTS && current_layer < layers.size() - 1){
            findMinPoint(layers, points, ++current_layer, maxDistanceTwoPoints, MIN_POINTS);
        }

    } else {
        if(size_points == 0){
            if(points.size() < MIN_POINTS && current_layer < layers.size() - 1){
                findMinPoint(layers, points, ++current_layer, maxDistanceTwoPoints, MIN_POINTS);
            }
        }
    }

}

//! find cluster of points (Lane object)
void pointsToLane(std::vector<Layer>    &layers,
                  std::vector<Lane>     &dst_lanes,
                  int                   H,
                  int                   minPoints,
                  int                   minDistanceTwoPoints){

    // convert all points of layers to original coornidations;
    for(Layer &l : layers){
       l.toOrigin();
    }
    // find clusters of points
    for(unsigned int i = 0; i < layers.size(); i++){
        unsigned int size_points = layers[i].points.size();

        if(size_points > 0){
            for(unsigned int j = 0; j < size_points; j++){

                std::vector<cv::Point> x;
                x.push_back(layers[i].points[j]);

                findRelatedPoint(layers, x, i+1, minDistanceTwoPoints);

                if(x.size() >= minPoints){
                    dst_lanes.push_back(Lane(x, H, 0));
                }
            }
        }
    }
}

//! find cluster of points (Lane object)
void pointsToLaneWithoutOrigin(std::vector<Layer>    &layers,
                  std::vector<Lane>     &dst_lanes,
                  int                   H,
                  int                   minPoints,
                  int                   minDistanceTwoPoints){
    cv::Mat a = cv::Mat::zeros(cv::Size(400, 320), CV_8UC1);
    // find clusters of points
    for(unsigned int i = 0; i < layers.size() - 2; i++){
        unsigned int size_points = layers[i].points.size();

        if(size_points > 0){
            for(unsigned int j = 0; j < size_points; j++){

                std::vector<cv::Point> x;
                x.push_back(layers[i].points[j]);

                findMinPoint(layers, x, i+1, minDistanceTwoPoints, minPoints);
                findRelatedPoint(layers, x, i+1, minDistanceTwoPoints);
                if(x.size() >= minPoints){
                    dst_lanes.push_back(Lane(x, H, 0));
                }
            }
        }
    }
}


//! find cluster of points (Lane object)
void pointsToLaneWithoutOriginAngle(std::vector<Layer>    &layers,
                               std::vector<Lane>     &dst_lanes,
                               int                   H,
                               int                   minPoints,
                               int                   minDistanceTwoPoints){
    // find clusters of points
    for(unsigned int i = 0; i < layers.size() - 2; i++){
        unsigned int size_points = layers[i].points.size();

        if(size_points > 0){
            for(unsigned int j = 0; j < size_points; j++){

                std::vector<cv::Point> x;
                x.push_back(layers[i].points[j]);
                findMinPoint(layers, x, i+1, minDistanceTwoPoints, minPoints);
                findRelatedPointAngle(layers, x, i+1, minDistanceTwoPoints);
                if(x.size() >= minPoints){
                    dst_lanes.push_back(Lane(x, H, 0));
                }
            }
        }
    }
}


void groupPoints(std::vector<Layer>     &layers,
                 int                    MAX_GROUP_POINTS){

    for(Layer &l : layers){
        if(l.points.size() > 1){
            for(int i = 0; i < l.points.size() - 1 ; i++){
                if(distanceTwoPoints(l.points[i], l.points[i+1]) <= MAX_GROUP_POINTS){
                    l.points[i].x = (l.points[i].x + l.points[i+1].x)/2;
                    l.points[i].y = (l.points[i].y + l.points[i+1].y)/2;
                    l.points.erase(l.points.begin() + i + 1);
                    i--;
                }
            }
        }

    }
}
void point2d2Bird(std::vector<cv::Point>   &input,
                  std::vector<cv::Point>     &output,
                  double X_RATIO,
                  double Y_RATION,
                  int X_ROI,
                  int Y_ROI) {

    std::vector<cv::Point2f> point2f, tmp_out;
    for(cv::Point &p : input){
        point2f.push_back(cv::Point(int(p.x * X_RATIO) + X_ROI , int(p.y * Y_RATION) + Y_ROI));
    }

    perspectiveTransform(point2f, tmp_out, imp::inverMatrixWrap);

    for(cv::Point2f &p : tmp_out){
        output.push_back(cv::Point((int)p.x, (int)p.y));
    }
}

void point2d2Bird(std::vector<cv::Point>    &input,
                  std::vector<cv::Point>    &output,
                  double                    X_RATIO,
                  double                    Y_RATION){
    std::vector<cv::Point2f> point2f, tmp_out;

    for(cv::Point &p : input){
        point2f.push_back(cv::Point(int(p.x * X_RATIO), int(p.y * Y_RATION)));
    }

    perspectiveTransform(point2f, tmp_out, imp::inverMatrixWrap);
    for(cv::Point2f &p : tmp_out){
        output.push_back(cv::Point((int)p.x , (int)p.y));
    }
}

void singPoint2d2Bird(cv::Point &point_src,
                      cv::Point &point_dst,
                      double    X_RATIO,
                      double    Y_RATION){
    std::vector<cv::Point2f> point2f(1), tmp_out(1);

    point2f[0].x = (float) (point_src.x * X_RATIO);
    point2f[0].y = (float) (point_src.y * Y_RATION);

    perspectiveTransform(point2f, tmp_out, imp::inverMatrixWrap);

    point_dst.x = (int)tmp_out[0].x;
    point_dst.y = (int)tmp_out[0].y;
}

void singPointBirdTo2d(cv::Point &point_src,
                       cv::Point &point_dst,
                       double    X_RATIO,
                       double    Y_RATION){
    std::vector<cv::Point2f> point2f(1), tmp_out(1);

    point2f[0].x = (float) (point_src.x * X_RATIO);
    point2f[0].y = (float) (point_src.y * Y_RATION);

    perspectiveTransform(point2f, tmp_out, imp::inverMatrixWrap.inv());

    point_dst.x = (int)tmp_out[0].x;
    point_dst.y = (int)tmp_out[0].y;
}


void pointBirdTo2d(std::vector<cv::Point>   &input,
                  std::vector<cv::Point>    &output,
                  double                    X_RATIO,
                  double                    Y_RATION){
    std::vector<cv::Point2f> point2f, tmp_out;

    for(cv::Point &p : input){
        point2f.push_back(cv::Point(int(p.x * X_RATIO), int(p.y * Y_RATION)));
    }

    perspectiveTransform(point2f, tmp_out, imp::inverMatrixWrap.inv());
    for(cv::Point2f &p : tmp_out){
        output.push_back(cv::Point((int)p.x , (int)p.y));
    }
}

