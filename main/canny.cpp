#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <config.h>
#include <getparam.h>
#include <laneimageprocesgreen.h>

using namespace cv;
using  namespace std;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold = 30;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
std::string window_name = "window";
int scale = 1;
int delta = 0;
int ddepth = CV_16S;
int thres = 130;

int h_low, s_low, v_low;
int h_high, s_high, v_high;

void CannyThreshold(int, void*)
{
    cv::Mat brg, m;
    cv::cvtColor(src, brg, cv::COLOR_RGB2BGR);
    cv::inRange(brg, cv::Scalar(0, 0, 150), cv::Scalar(255, 255, 255), m);


    cv::imshow("src", brg);
    cv::Mat roi, bird, grad_x, grad_y, abs_grad_y, abs_grad_x, grad, gray_sob;
    roi = src(imp::roi).clone();
    cv::cvtColor( roi, src_gray, CV_RGB2GRAY );
    /// Reduce noise with a kernel 3x3
    cv::blur( src_gray, detected_edges, Size(3,3) );

    /// Canny detector
    cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

    /// Using Canny's output as a mask, we display our result
    dst = Scalar::all(0);

    src_gray.copyTo(dst, detected_edges);


    std::vector<cv::Vec4i> hierarchy;
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dst, contours, hierarchy, CV_RETR_LIST, CHAIN_APPROX_SIMPLE);

    cv::Mat mask = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Scalar color(255);
    for( int i = 0; i< contours.size(); i++ )
    {

    }
    drawContours( mask, contours, -1, color, 5, 16);



    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );


    int dilation_size = 3;
    Mat element = getStructuringElement( cv::MORPH_ELLIPSE,
                            Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                    Point( dilation_size, dilation_size ) );

    birdView(detected_edges, bird);
//    blur(bird, bird, cv::Size(5,5));
    dilate( bird, bird, element);
//    threshold(bird,bird, thres, 255, cv::THRESH_BINARY );
//

    cv::imshow( "adf", bird );
    cv::imshow( window_name, detected_edges );
}

int testVideo(std::string filename){

    cv::namedWindow(window_name);
    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
    createTrackbar( "thres", window_name, &thres, 255, CannyThreshold);
    createTrackbar("h_low", window_name, &h_low, 255, CannyThreshold);
    createTrackbar("s_low", window_name, &s_low, 255, CannyThreshold);
    createTrackbar("v_low", window_name, &v_low, 255, CannyThreshold);
    createTrackbar("h_high", window_name, &h_high, 255, CannyThreshold);
    createTrackbar("s_high", window_name, &s_high, 255, CannyThreshold);
    createTrackbar("v_high", window_name, &v_high, 255, CannyThreshold);



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

    while(true){
        st = getTickCount();

        video >> src;

        if(src.empty()){
            std::cout << "Failed read frame" << std::endl;
            break;
        }

        cv::Mat hsv, gray, bgr;

        num_frames ++;

        CannyThreshold(0, 0);

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
    testVideo("/home/nc/Desktop/video/24.avi");

    return 0;
}