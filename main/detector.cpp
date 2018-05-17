#include "stdafx.h"
#include "detectorobj.h"
#include <iostream>

using namespace objdetect;
int main() {
    std::cout << "K\n";
    std::string rgb_file = "C:/Users/Binh Bum/Dropbox/prototype/Video/5/2.avi";

    cv::Mat color_frame;

    cv::VideoCapture cap_color(rgb_file);
    if (!cap_color.isOpened()) {
	   std::cout << "Error when reading avi stream\n";
	   return 1;
    }
    Detector_Obj detector = Detector_Obj();
    detector.insert_filter(cv::Scalar(111, 188, 52), cv::Scalar(120, 255, 123));
    detector.insert_filter(cv::Scalar(16, 59, 229), cv::Scalar(24, 93, 255));
    double freq = cv::getTickFrequency();
    double st = 0, et = 0, fps = 0;
    double sum_fps = 0;
    int num_frames = 0;

    while (1) {
	   st = cv::getTickCount();
	   cap_color >> color_frame;
	   num_frames++;
	   if (color_frame.empty())
		  break;
	   bool exist = false;

	   //	   std::vector<cv::Rect> all_rects = detector.find_all_rect(color_frame, 0.5);
	   std::vector<cv::Point2f> points;
		// detector.get_two_bottom_points_depth(color_frame, points);
	   for (int i = 0; i < points.size(); ++i)
	   {
		  //		   cv::rectangle(color_frame, all_rects[i],cv::Scalar(0,255,0),2 );
		  //		    std::cout << all_rects[i].area()<<std::endl;
		  cv::line(color_frame, points[0], points[1], cv::Scalar(0, 255,0), 2);
		  exist = true;
	   }

	   cv::imshow("test", color_frame);

	   if (exist)
	   {
		  cv::waitKey(1);
	   }
	   else {
		  cv::waitKey(1);
	   }

	   et = cv::getTickCount();
	   sum_fps += 1.0 / ((et - st) / freq);
	   	   std::cout << "FPS: " << sum_fps / num_frames << std::endl;

    }
    cv::waitKey(0);
    return 0;
}
