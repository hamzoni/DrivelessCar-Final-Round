#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <config.h>
#include <getparam.h>

#include <trafficsign.h>


using namespace cv;
using  namespace std;



int testVideo(std::string filename){
    cv::VideoCapture video(filename);

    if(!video.isOpened()){
        cout << "OPEN VIDEO FAILED!" << endl;
        return -1;
    }

    double total_time = 0;
    int num_frames = 0;

    double freq = getTickFrequency();
    double st = 0, et = 0, fps = 0;
    double sum_fps = 0;
    cv::Mat frame;


    while(true){
        st = getTickCount();

        video >> frame;

        if(frame.empty()){
            std::cout << "Failed read frame" << std::endl;
            break;
        }

        cv::Mat hsv, gray, bgr;
        cv::cvtColor(frame, bgr, cv::COLOR_RGB2BGR);
        cv::cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);

        sign::findTraffic.find(hsv, gray);
        cv::imshow("window", frame);

        num_frames ++;

        int k = waitKey(conf::WAIT_KEY) & 0xff;

        if(k == 27){
            break;
        }
        if(k == 32){
            waitKey();
        }

        et = getTickCount();
        sum_fps += 1.0 / ((et-st)/freq);
        cerr << "FPS: "<< sum_fps/num_frames << '\n';

    }
    video.release();
    destroyAllWindows();
    return 1;
}



int main()
{

    init("/home/nc/Desktop/prototype");
    sign::findTraffic.init("/home/nc/Desktop/prototype/sign/train.txt");
    testVideo("/home/nc/Desktop/video/25.avi");
    return 0;
}