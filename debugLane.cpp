#include "t.h"
#include "api_i2c_pwm.h"
#include <iostream>
#include "Hal.h"
#include <config.h>
#include <getparam.h>
#include <frameprocess.h>
#include "secret.h"
#include <thread>
#include <carconf.h>
#include <detectorobj.h>
#include <haar.h>
#include <X11/Xlib.h>
#include <RTIMULib.h>
#include "carconf.h" // config speed button red green

using namespace openni;
using namespace EmbeddedFramework;

PCA9685 *pca9685 = new PCA9685() ;
GPIO *gpio = new GPIO();
I2C *i2c_device = new I2C();
void setup(PCA9685 *pca9685);


void show_lane(){
        try{
            cv::Mat img_lane;
            rgb.copyTo(img_lane);
            Rect object_rect;
            //Point2f object_points[4];
//             cv::imshow("original_depth",depthCam);
            status::OBJ = objdetect::detector.get_object_depth(depthCam, img_lane, object_rect);
            //cout<< "Fast Check object : "<<status::OBJ<<"\n";
            updateObj(object_rect, 2, 2);
            cv::resize(img_lane, img_lane, cv::Size(conf::WIDTH, conf::HEIGHT));
            processImg(img_lane);
            //processImgSe(img_lane);
            //std::cout<< "angle: " << status::ANGLE << std::endl;

            api_set_STEERING_control(pca9685 , status::ANGLE);
        } catch (exception &e){}
}

void thread_read_color(){
        cv::Mat imgColor = get_color();
        if (conf::WRITE_VIDEO) {
            if (!imgColor.empty())
                color_videoWriter.write(imgColor);
        }
        cv::imshow("color", imgColor);
        cv::waitKey(1);
        imgColor.copyTo(rgb);
}

void thread_read_depth(){
        cv::Mat imgDepth = get_depth();
        if (conf::WRITE_VIDEO) {
            if (!imgDepth.empty())
                depth_videoWriter.write(imgDepth);
        }
//        cv::imshow("depth", imgDepth);
//        cv::waitKey(1);
        imgDepth.copyTo(depthCam);
}


int main( int argc, char* argv[] ) {

    initCam();
    XInitThreads();
    init("/home/ubuntu/Desktop/prototype");
    setup(pca9685);
    char key = 0;
    while (true){

        key = getkey();
        if(key == '9'){
            color.destroy();
            depth.destroy();
            openni::OpenNI::shutdown();

            ////////  Release //////////////////////////////////////////////////////
            if(conf::WRITE_VIDEO){
                color_videoWriter.release();
                depth_videoWriter.release();
            }
            break;
        }

        if(1){
            thread_read_color();
            thread_read_depth();
            cv::waitKey(1);
            show_lane();
            cerr << "FPS -- : " << " " << status::ANGLE << '\n';
        }
    }
    return 0;
}

void setup(PCA9685 *pca9685){
    int inti_throttle = 0;
    double inti_angle = 0;
    api_pwm_pca9685_init( pca9685);
    if (pca9685->error >= 0){
        api_set_FORWARD_control( pca9685,inti_throttle);
        api_set_STEERING_control(pca9685, inti_angle);
    }
}

