#include "haar.h"

cv::VideoCapture capture;
int frameid = 0;
int nnn = 0;
int main(){
 	capture.open("3.avi");
    const char * WINDOW_NAME = "Detection";
    cv::namedWindow(WINDOW_NAME, 1);
    int st, end, frame = 0;
    double freq = cv::getTickFrequency();
    int conf= 0;
    signhaar::haar.init("/home/q/Desktop/haar/models");
    for (;;)
    {
        frameid++;
        st = cv::getTickCount();
        cv::Mat frame, frame_cp, out;
        nnn++;
        capture >> frame;
        cv::flip(frame, frame,1);
        // cv::cvtColor(frame , frame, cv::COLOR_BGR2GRAY);
        out = signhaar::haar.crop_h(frame);
        int flag = signhaar::haar.detect_haar(out, frameid);
        //std::cout<< "size: ----" <<out.size()<<  std::endl;
        signhaar::haar.dis_stop(out);
        cv::imshow(WINDOW_NAME, out);
        end = cv::getTickCount();
        int k = cv::waitKey(50) & 0xff;
        if(k == 27){
            break;
        }
        if(k == 32){
            cv::waitKey();
        }
        std::cout <<"fps : " << 1/double((end - st)/freq ) << "turn ----- "<<  conf  << std::endl;
    }
    return 0;
}

