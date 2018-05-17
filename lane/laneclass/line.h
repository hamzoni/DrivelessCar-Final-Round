#ifndef LINE_H
#define LINE_H

#include "opencv2/highgui/highgui.hpp"

#define ESP 1e-8
#define PI 3.14159

// Line class declaration

class Line{
public:
    double slope, angle;

    // y = slope*x + c
    double c;

    cv::Point start;
    cv::Point end;
    cv::Point anchor;

    Line();

    Line(double, cv::Point, int , int);
    Line(cv::Point, cv::Point);


    template <typename T> T getX(T);
    template <typename T> T getY(T);
    template <class P> double getDisSide(P);

    // toString
    friend std::ostream& operator << (std::ostream& out, const Line line){
        out << "(" << line.start << " , " << line.end << ")" << "  " << line.slope << " " << line.c << std::endl;
        return out;
    }
};

template <typename T>
T Line::getX(T Y) {
    return (T)((Y - this->c) / this->slope);
}

template <typename T>
T Line::getY(T X) {
    return (T)(X*this->slope + this->c);
}

template <class P>
double Line::getDisSide(P p){
    return (double)(p.x * this->slope - p.y + this->c);
}
#endif // LINE_H
