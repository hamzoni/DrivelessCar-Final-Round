#ifndef DETECTOR_H
#define DETECTOR_H
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#define VIDEO_FRAME_WIDTH 160
#define VIDEO_FRAME_HEIGHT 120
#define DIFF_CAM 5

class Detector_Obj
{

public:
    Detector_Obj();
	cv::Rect get_color_rect(cv::Mat color, cv::Rect maxsize_rect, int maxsize);

	//bool get_object_depth(cv::Mat depthImg, cv::Mat color_, cv::Point2f object_rect[]);
	bool get_object_depth(cv::Mat depthImg, cv::Mat color_, cv::Rect &object_rect);
private:
   bool type_canny(cv::Mat depthImg,cv::Mat color_, cv::Rect &object_rect);
	bool type_roi_floor(cv::Mat depthImg,cv::Mat color_, cv::Rect &object_rect);
    std::vector<cv::Rect> get_objects(cv::Mat depth);
};

namespace objdetect{
    extern Detector_Obj detector;
}
#endif // DETECTOR_H
