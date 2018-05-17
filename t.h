
#ifndef TAB
#define TAB

#include "api_kinect_cv.h"
#include "opencv2/highgui.hpp"
#include <config.h>
#define HE 480
#define WI 640
#define SENSOR	165
#define BUTTON1	160
#define BUTTON2	161
#define BUTTON3	163
#define BUTTON4	164
#define LED		166
#define MPU_left_angle 5
#define MPU_left_comp 355

#define MPU_right_angle 12
#define MPU_right_comp 339

extern VideoStream depth;
extern VideoStream color;
extern VideoFrameRef frame_color;
extern VideoFrameRef frame_depth;
extern cv::Mat depthImg;
extern cv::Mat colorImg;
extern cv::Mat rgb;
extern cv::Mat depthCam;
extern VideoWriter color_videoWriter;
extern VideoWriter depth_videoWriter;

Mat get_depth();
Mat get_color();
void analyzeColor(const VideoFrameRef& frame_color, cv::Mat& color_img);
void analyzeDepth(const VideoFrameRef &frame_depth, cv::Mat& depth_img);
int initCam();

#endif
