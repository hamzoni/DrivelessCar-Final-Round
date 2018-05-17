//
// Created by nc on 14/04/2018.
//
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "iostream"
#define SIMD_OPENCV_ENABLE

#include "Simd/SimdLib.hpp"
#include "Simd/SimdContour.hpp"
#include "Simd/SimdDrawing.hpp"
#include "Simd/SimdPoint.hpp"

void processImg(){

}
int testVideoGreen(std::string filename){

    typedef Simd::ContourDetector<Simd::Allocator> ContourDetector;
    ContourDetector detector;
    ContourDetector::Contours contours;
    ContourDetector::View image, mask2;

    typedef Simd::Point<ptrdiff_t> Point;
    Point size  = cv::Size(640, 480);
    detector.Init(size);

    ::cvNamedWindow("hello", 1);
    const std::string SIMD_DEBUG_WINDOW_NAME = "/home/nc/Desktop/video/6.avi";
    ::CvCapture * capture = ::cvCreateFileCapture(SIMD_DEBUG_WINDOW_NAME.c_str());
    int num_frames = 0;

    double freq = cv::getTickFrequency();
    double st = 0, et = 0, fps = 0;
    double sum_fps = 0;

    while (1){
        st = cv::getTickCount();
        ::IplImage *frame = ::cvQueryFrame(capture);
        if(!frame){
            break;
        }




        image = ContourDetector::View(frame->width, frame->height, frame->widthStep, ContourDetector::View::Bgr24, frame->imageData);
        ContourDetector::View gray(frame->width, frame->height, ContourDetector::View::Gray8);
        Simd::BgraToGray(image, gray);
        detector.Detect(gray, contours);

        std::cout << contours.size() << std::endl;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            std::cout << i << std::endl;
            for (size_t j = 1; j < contours[i].size(); ++j)
                Simd::DrawLine(image, contours[i][j - 1], contours[i][j], uint8_t(255));
        }
//        ::cvShowImage("hello", image.data());


        char c = cvWaitKey(1);
        if(c == 27){
            break;
        }


        num_frames ++;
        et = cv:: getTickCount();
        sum_fps += 1.0 / ((et-st)/freq);
        std::cerr << "FPS: "<< sum_fps/num_frames << '\n';

    }

    ::cvReleaseCapture(&capture);
    ::cvDestroyWindow(SIMD_DEBUG_WINDOW_NAME.c_str());






    return true;

}


int main(){

    testVideoGreen("");
}