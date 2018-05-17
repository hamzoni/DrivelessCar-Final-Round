#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;

class Parabol{
    // ax**2 + bx + c = 0
public:
    double a, b, c, d;

    Parabol(double a, double b, double c){
        this->a = a;
        this->b = b;
        this->c = c;
    }
};

void drawParabol(cv::Mat src, Parabol p, std::vector<cv::Point> points){
    cout << p.a << p.b << endl;
    int width = 500;
    cv::Mat image = cv::Mat::zeros(width,width,CV_8UC3);

    for(cv::Point &p : points){
        cv::circle(image, p, 3, cv::Scalar(0,255,255), -1);
    }
    vector<cv::Point> list_point(width);
    for(int i = 0; i < width; i++){
        list_point[i].x = i;
        list_point[i].y = p.a * i * i * i + p.b * i * i + p.c * i + p.d;
    }
//Draw the curve
    for(int i = 1; i < width; i++){
        cv::line(image,list_point[i-1],list_point[i],cv::Scalar(255,255,255),2);
    }
    imshow("image",image);
    cv::waitKey();
}

void fitParabol(std::vector<cv::Point> points, Parabol &parabol){
    double x = 0, y = 0, xy = 0, x2 = 0, x2y = 0, x3 = 0, x4 = 0;
    int z1 = points[2].y, z2 = points[3].x;
    for(cv::Point p : points){
        p.x -= z2;
        p.y -= z1;
        x += p.x;
        y += p.y;
        xy += p.x * p.y;
        x2 += p.x * p.x;
        x2y += x2 * p.y;
        x3  += x2 * p.x;
        x4 += x2 * x2;
    }
    int n_points = points.size();

    double a = n_points, b = x, c = x2, d = -y;
    double l = x, m = x2, n = x3, k = -xy;
    double p = x2, q = x3, r = x4, s = -x2y;

    std::cout << a << " " << b << " " << c << " " << d << std::endl;
    std::cout << l << " " << m << " " << n << " " << k << std::endl;
    std::cout << p << " " << q<< " " << r << " " << s << std::endl;


    double D = (a*m*r+b*p*n+c*l*q)-(a*n*q+b*l*r+c*m*p);
    double x_ = ((b*r*k+c*m*s+d*n*q)-(b*n*s+c*q*k+d*m*r))/D;
    double y_ = ((a*n*s+c*p*k+d*l*r)-(a*r*k+c*l*s+d*n*p))/D;
    double z_ = ((a*q*k+b*l*s+d*m*p)-(a*m*s+b*p*k+d*l*q))/D;

    parabol.a = z_;
    parabol.b = (-2*parabol.a*z2 + y_);
    parabol.c = -y_*z2 + x_ + z_*z2*z2 + z1;
    parabol.d = -xy;

    cout << x_ << " " << y_ << " " << z_ <<endl;
}

void three_equation(){
    double a = 3, b = 2, c = 2, d = -9;
    double l = 2, m = 6, n = 7, k = -1;
    double p = 12, q = 4, r = 12, s = -2;


    double D = (a*m*r+b*p*n+c*l*q)-(a*n*q+b*l*r+c*m*p);
    double x = ((b*r*k+c*m*s+d*n*q)-(b*n*s+c*q*k+d*m*r))/D;
    double y = ((a*n*s+c*p*k+d*l*r)-(a*r*k+c*l*s+d*n*p))/D;
    double z = ((a*q*k+b*l*s+d*m*p)-(a*m*s+b*p*k+d*l*q))/D;


cout << x << " " << y << " " << z <<endl;

}
int main(){
//    three_equation();

    Parabol parabol(1,-1,-2);
    cv::Mat src;
//
//    drawParabol(src, p);

    vector<cv::Point> p;
    p.push_back(cv::Point(179, 197));
    p.push_back(cv::Point(189, 188));
    p.push_back(cv::Point(201, 179));
    p.push_back(cv::Point(214, 171));
    p.push_back(cv::Point(226, 162));
    p.push_back(cv::Point(237, 153));
    p.push_back(cv::Point(247, 144));

    fitParabol(p, parabol);


    drawParabol(src, parabol, p);

}