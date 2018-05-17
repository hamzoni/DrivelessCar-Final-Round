//
// Created by nc on 20/04/2018.
//

#include "iostream"
#include "testnamesapceapi.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "math.h"

using namespace std;
using namespace cv;

void drawPara(cv::Size t,
              std::vector<double> coef,
              std::vector<double> xV,
              std::vector<double> yV ){

    cv::Mat mask = cv::Mat::zeros(t, CV_8UC1);

    std::vector<cv::Point2f> list_point(t.width);
    for(int x = 0; x < t.width; x++){
        double xa = (double)x;
        double y = coef[0] + coef[1] * xa + coef[2] * xa * xa;
        list_point[x].x = x;
        list_point[x].y = y;
    }

    for(int i = 0; i < xV.size(); i++){
        cv::circle(mask, cv::Point(xV[i], yV[i]), 3, cv::Scalar(255,255,255), 2);
    }

    for(int i = 1; i < t.width; i++){
        cv::line(mask,list_point[i-1],list_point[i],Scalar(255,255,255), 1);
    }

    cv::imshow("para", mask);
    cv::waitKey();

}

int main(){
    const unsigned int order = 2;
    unsigned int countOfElements = 3;
    int result;
    std::vector<double> dependentValues, independentValues, coef;
    dependentValues.push_back(9); independentValues.push_back(313);
    dependentValues.push_back(11); independentValues.push_back( 305);
    dependentValues.push_back(12); independentValues.push_back( 292);
    dependentValues.push_back(15); independentValues.push_back( 284);
    dependentValues.push_back(19); independentValues.push_back( 273);
    dependentValues.push_back(21); independentValues.push_back(265);
    dependentValues.push_back(25); independentValues.push_back( 255);
    dependentValues.push_back(29); independentValues.push_back( 244);
    dependentValues.push_back(33); independentValues.push_back( 233);
    dependentValues.push_back(36); independentValues.push_back( 222);
    dependentValues.push_back(40); independentValues.push_back( 212);
    dependentValues.push_back(43); independentValues.push_back( 203);
    dependentValues.push_back(47); independentValues.push_back( 193);
    dependentValues.push_back(49); independentValues.push_back( 185);
    dependentValues.push_back(53); independentValues.push_back( 175);
    dependentValues.push_back(57); independentValues.push_back( 165);
    dependentValues.push_back(60); independentValues.push_back( 156);
    dependentValues.push_back(63); independentValues.push_back( 147);
    dependentValues.push_back(67); independentValues.push_back( 135);
    dependentValues.push_back(71); independentValues.push_back( 124);
    dependentValues.push_back(74); independentValues.push_back( 115);
    dependentValues.push_back(77); independentValues.push_back( 104);
    dependentValues.push_back(71); independentValues.push_back( 92);


    coef = polyfit(dependentValues, independentValues, 2);
    for(double a: coef){
        std::cout<< a << " ";
    } std::cout << endl;



    drawPara(cv::Size(320,320), coef, dependentValues, independentValues);
}