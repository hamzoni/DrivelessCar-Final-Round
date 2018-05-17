#include "getparam.h"
#include "frameprocess.h"
#include "green.h"
#include "haar.h"
#include "secret.h"

int frameid = 0;
// test image processing from video
int testVideo(std::string filename){

    cv::VideoCapture video(filename);

    if(!video.isOpened()){
        std::cout << "OPEN VIDEO FAILED!" << std::endl;
        return -1;
    }

    double total_time = 0;
    int num_frames = 0;

    double freq = cv::getTickFrequency();
    double st = 0, et = 0, fps = 0;
    double sum_fps = 0;

    while(true){
        st = cv::getTickCount();

        cv::Mat frame;
        video >> frame;

        if(frame.empty()){
            std::cout << "Failed read frame" << std::endl;
            break;
        }


        frameid++;
        cv::Mat signImg, img_con;
        frame.copyTo(signImg);
        cv::Mat imgs_signs =  signhaar::haar.crop_h(signImg,590,180);
        img_con = imgs_signs.clone();
        double contrast = 3.0; // A
        int brightness = 30; // B
        for (int indexRow = 0; indexRow < imgs_signs.rows; indexRow++) {
            for (int indexCol = 0; indexCol < imgs_signs.cols; indexCol++) {
            for (int channel = 0; channel < 3; channel++) {
                img_con.at<cv::Vec3b>(indexRow, indexCol)[channel] = cv::saturate_cast<uchar>
                       (contrast*(img_con.at<cv::Vec3b>(indexRow, indexCol)[channel]) + brightness);
                }
            }
        }
        int id = signhaar::haar.detect_haar(img_con, frameid);

        if (id == 1) {
            status::LEFT = true;
            std::cout << "Turn Right" << std::endl;
        }
        if (id == 2) {
            status::LEFT = false;
            std::cout << "Turn Left" << std::endl;
        }

        cv::resize(frame, frame, cv::Size(conf::WIDTH, conf::HEIGHT));
        num_frames ++;
        processImgSe(frame);

        int k = cv::waitKey(conf::WAIT_KEY) & 0xff;

        if(k == 27){
            break;
        }
        if(k == 32){
            cv::waitKey();
        }

        et = cv::getTickCount();
        sum_fps += 1.0 / ((et-st)/freq);
        std::cerr << "FPS: "<< sum_fps/num_frames << '\n';

    }
    video.release();
    cv::destroyAllWindows();
    return 1;
}

int main(){
    init("/home/ubuntu/Desktop/prototype");
    testVideo("/home/ubuntu/Desktop/69.avi");
    signhaar::haar.init("/home/ubuntu/Desktop/prototype/haar/models");
}
