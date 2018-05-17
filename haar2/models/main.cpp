#include "haar.h"

cv::VideoCapture capture;
int frameid = 0;
int nnn = 0;
int main(){
 	capture.open("80.avi");
    const char * WINDOW_NAME = "Detection";
    cv::namedWindow(WINDOW_NAME, 1);
    int st, end, frame = 0;
    double freq = cv::getTickFrequency();
    int conf= 0;
    Haar haar("/home/q/Desktop/haar/models");
    for (;;)
    {
        frameid++;
        st = cv::getTickCount();
        cv::Mat frame, frame_cp, out, imageHsv, img_con;
        nnn++;
        capture >> frame;
        cv::flip(frame, frame,1);
        img_con = frame.clone();
        double contrast = 2.0; // A
        int brightness = 27; // B
        for (int indexRow = 0; indexRow < frame.rows; indexRow++) {
            for (int indexCol = 0; indexCol < frame.cols; indexCol++) {
                for (int channel = 0; channel < 3; channel++) {
                    img_con.at<cv::Vec3b>(indexRow, indexCol)[channel] = cv::saturate_cast<uchar>
                           (contrast*(img_con.at<cv::Vec3b>(indexRow, indexCol)[channel]) + brightness);
                }
    	    }
        }
        // cvtColor(frame, imageHsv, CV_BGR2HSV);
        // std::vector<cv::Mat> hsvChannels;
        // cv::split(imageHsv, hsvChannels);
        // cv::equalizeHist(hsvChannels[2], hsvChannels[2]);
        // cv::merge(hsvChannels, imageHsv);
        // cv::cvtColor(imageHsv, frame, CV_HSV2BGR);
        // cv::cvtColor(frame , frame, cv::COLOR_BGR2GRAY);
        out = haar.crop_h(img_con);
        int flag = haar.detect_haar(out, frameid);
        //std::cout<< "size: ----" <<out.size()<<  std::endl;
        //haar.dis_stop(out);
        cv::imshow(WINDOW_NAME,out);

        //cv::imshow(WINDOW_NAME, frame);
        end = cv::getTickCount();
        int k = cv::waitKey(50) & 0xff;
        if(k == 27){
          cv::imwrite("train_data/left/n/nleft_1"+ std::to_string(nnn)+ ".jpg", frame);
          cv::waitKey();
        }
        if(k == 32){
            cv::imwrite("train_data/left/p/pleft_1"+ std::to_string(nnn)+ ".jpg", frame);
            cv::waitKey();
        }
        std::cout <<"fps : " << 1/double((end - st)/freq ) << "turn ----- "<<  conf  << std::endl;
    }
    return 0;
}
