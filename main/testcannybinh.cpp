#include "stdio.h"
#include "iostream"
#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

cv::Mat image;
Scalar lower(255, 255, 255);
Scalar upper(0, 0, 0);
cv::Mat hsv;

void show_mask()
{
    cv::Mat mask;
    
    cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);
    cv::inRange(hsv, lower, upper, mask);
//    imshow("hsv", hsv);
    cv::imshow("mask", mask);
    cv::imshow("image", image);
}
void range_grow(Scalar current)
{
   
    for (int i = 0; i < 4; ++i)
    {
	   if (lower[i] > current[i])
	   {
		  lower[i] = current[i];
	   }
	   if (upper[i] < current[i])
	   {
		  upper[i] = current[i];
	   }
    }
	std::cout << " [ Current ]  : " << current << endl;
}
void box_click(int event, int x, int y, int flags, void*userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
	   cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);

	   Vec3b color = hsv.at<Vec3b>(Point(x, y));
	   range_grow(color);
	   std::cout << "Lower : " << lower <<endl;
	   std::cout << "Upper : " << upper<<endl;
	   std::cout << "----------------------"<<endl;
    }
}



void run(VideoCapture cap)
{
    bool read = true;
    cv::Mat frame;
    while (true)
    {

	   if (read)
	   {
		  cap >> frame;
//		  cvtColor(frame, frame, COLOR_RGB2BGR);
	   }
	   
	   image = frame.clone();

	   cv::setMouseCallback("image", box_click, NULL);
	   cv::setMouseCallback("mask", box_click, NULL);
	   show_mask();
	   int k = cv::waitKey(20);
	   if (k == 32)
	   {
		  read = !read;
	   }
	   if(k == 27){
	   	break;
	   }
	   else if (k == 'r')
	   {
		  lower =  Scalar(255, 255, 255);
		  upper = Scalar(0, 0, 0);
	   }
    }
}

int main() {
	cv::VideoCapture cap("/media/nc/HOADX/color.avi");
    if (!cap.isOpened()) 
	   return -1;
    cv::namedWindow("image");
    namedWindow("mask");
    run(cap);

}
