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
RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
RTIMU *imu = RTIMU::createIMU(settings);
bool status_break = false;
int pid;
int speed = 0;
bool run = false;
bool process = false;
int dis_stop;
double start = car::MIN_SPD;
int stop_d =0, status_stop = 1;
int count_=0, summ= 0;
bool SSENSOR = false;
long frameid= 0;
int decrease_stop= 0;
unsigned int bt_status_1 = 1, bt_status_2= 1;
bool checkPoint = false;
bool run_mpu = false;
int count_check_point = 0;
float mpu_value= 0;
int mpu_speed = 0;
long frame_id = 0;
void stop();
void brake_();
void setup(PCA9685 *pca9685);


void thread_mpu(){
    while(1){
        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            //std::cout<< "mpu ----------" << imuData.compass.length()<<  std::endl;
            mpu_value = signhaar::haar.getDir(imuData.compass.y(), imuData.compass.x());
            //std::cout<< "mpu ----------" << imuData.fusionPose.z()*180/3.14 <<  std::endl;
            std::cout << signhaar::haar.getDir(imuData.compass.y(), imuData.compass.x()) << std::endl;
            std::cout<< "mpu angle: ------------------------   "<< (int)imuData.compass.x() <<"   "<<  (int)imuData.compass.y()<< "  "<< (int)imuData.compass.z()<< std::endl;
            float mpu_y = imuData.compass.y();
            float mpu_x = imuData.compass.x();

            if(std::abs(mpu_value - 10) <= 10 && run_mpu){
                checkPoint = true;
            } else {
                checkPoint = false;
            }
        }
    }
}

void thread_sensor(){
    while(1){
        unsigned int sensor_status = 0;
		gpio->gpioGetValue(SENSOR, &sensor_status);
        if(count_ % 25 ==0){
            count_ = 0;
            if(summ <=3){
                SSENSOR = true;
            }
            else SSENSOR = false;
            summ = 0;
        }

        summ += sensor_status;
        count_++;
    }
}

void thread_control(double angle, int throttle){
    api_set_STEERING_control(pca9685 , angle);
    api_set_FORWARD_control( pca9685 , throttle);
}

void show_stop(){
    while(1){
        try{
            cv::Mat stopImg;
            rgb.copyTo(stopImg);
            cv::Mat imgs_stop =  signhaar::haar.crop_h(stopImg,640,150);
            cv::flip(imgs_stop, imgs_stop, 1);
            dis_stop = signhaar::haar.dis_stop(imgs_stop);
            //    if(dis_stop==1)
            //        std::cout << "STOP" << std::endl;
            //    if(dis_stop ==2)
            //        std::cout<< "-----s-------------stop" << std::endl;
            // cv::imshow("stoppp", imgs_stop);

        } catch (exception &e){}
    }
}

void show_sign(){
    while(1){
        try{
            frameid++;
            cv::Mat signImg;
            rgb.copyTo(signImg);
            cv::Mat imgs_signs =  signhaar::haar.crop_h(signImg,640,150);
            int id = signhaar::haar.detect_haar(imgs_signs, frameid);
            if(id == 1 && pid == 2)status_stop = 2;
            if(id == 2 && pid ==1) status_stop = 1;
            if (id == 1) {
                status::LEFT = false;
                //std::cout << "Turn Right" << std::endl;
            }
            if (id == 2) {
                status::LEFT = true;
                //std::cout << "Turn Left" << std::endl;
            }
            // cv::imshow("signs", imgs_signs);
    //        cv::watiKey(1);
        } catch (exception &e){}
    }
}

void show_lane(){
    while(1){
        try{
            cv::Mat img_lane;
            rgb.copyTo(img_lane);
            Rect object_rect;
            //Point2f object_points[4];
            //status::OBJ = objdetect::detector.get_object_depth(depthCam, img_lane, object_rect);
            //cout<< "Fast Check object : "<<status::OBJ<<"\n";
            updateObj(object_rect, 2, 2);
            cv::resize(img_lane, img_lane, cv::Size(conf::WIDTH, conf::HEIGHT));
            processImg(img_lane);
            //processImgSe(img_lane);
            //std::cout<< "angle: " << status::ANGLE << std::endl;

            api_set_STEERING_control(pca9685 , status::ANGLE);
        } catch (exception &e){}
    }
}


bool psensor = false;
bool run_stop = false;

int control(char key, bool sensor){

    if(sensor == false && psensor){
        key = '2';
        psensor = false;

    }

    if(sensor && psensor == false){
        key = '4';
        psensor = true;
    }

    if(key == '2') {
        if(run == false) start = car::MIN_SPD;
        process = true;
        run = true;
        status_break = false;
        decrease_stop= 0;
        mpu_speed= 0;
    }
    if(key == '8'){
        run = false;
        process = false;
    }

    if(key == '4'){
        run = false;
        process = true;
    }

    if(run && start < car::START_SPD){
        start += car::STEP_START_SPD;
        return (int) start;
    }
//
//    if(dis_stop == 1){
//        run = false;
//        process = true;
//        stop_d = 1;
//    }

    speed = status::SPEED;

    if(dis_stop == 2){
        car::STEP_STOP_SPD = 5;
        decrease_stop = 1;
        run_mpu = true;
    }

    if(checkPoint){
        run = false;
        stop();
        run_mpu = false;
        stop_d = 1;
    }
    return speed;
}

void thread_read_color(){
    while(1){
        cv::Mat imgColor = get_color();
        if (conf::WRITE_VIDEO) {
            if (!imgColor.empty())
                color_videoWriter.write(imgColor);
        }
        imgColor.copyTo(rgb);
    }
}

void thread_read_depth(){
    while(1){
        cv::Mat imgDepth = get_depth();

        if (conf::WRITE_VIDEO) {
            if (!imgDepth.empty())
                depth_videoWriter.write(imgDepth);
        }
        cv::imshow("depth", imgDepth);
        cv::waitKey(1);
        imgDepth.copyTo(depthCam);
    }
}

int main( int argc, char* argv[] ) {
    initCam();
    XInitThreads();

    imu->IMUInit();
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
   // imu->m_sampleRate = 10;

    init("/home/ubuntu/Desktop/prototype");
    signhaar::haar.init("/home/ubuntu/Desktop/prototype/haar/models");

    setup(pca9685);

    FILE *LogFile = fopen("thetaLog.txt", "w");
	gpio->gpioExport(SENSOR);
	gpio->gpioExport(LED);
    gpio->gpioExport(BUTTON1);
    gpio->gpioExport(BUTTON2);

	gpio->gpioSetDirection(BUTTON1, INPUT);
    gpio->gpioSetDirection(BUTTON2, INPUT);

	gpio->gpioSetDirection(SENSOR, INPUT);
	gpio->gpioSetDirection(LED, OUTPUT);
    i2c_device->m_i2c_bus = 1;

    if (!i2c_device->HALOpen()) {
		printf("Cannot open I2C peripheral\n");
		exit(-1);
	} else printf("I2C peripheral is opened\n");


    char key = 0;
    double st = 0, et = 0, fps = 0, angle = 0;
    double freq = getTickFrequency();
    int throttle = 0;
    int frame_stop = 0;

    unsigned int last = 0, sum = 0, bt_last1 = 1, bt_last2 = 1;

    std::thread cam_color(&thread_read_color);
    std::thread cam_depth(&thread_read_depth);
    std::thread lane_(&show_lane);
    std::thread sign_(&show_sign);
    std::thread stop_(&show_stop);


    //std::thread stop_(&thread_stop);
    std::thread mpu_(&thread_mpu);
    cam_color.detach();
    cam_depth.detach();
    //mpu_.detach();
//    lane_.detach();

    //sign_.detach();
    //stop_.detach();

    while (true){
        frame_id++;
        unsigned int sensor_status = 0;
		gpio->gpioGetValue(SENSOR, &sensor_status);
		gpio->gpioGetValue(BUTTON1, &bt_status_1);
        gpio->gpioGetValue(BUTTON2, &bt_status_2);
		//std::cout<< "button1 :: ------------------"<<  bt_status_1 << std::endl;
        //std::cout<< "button2 :: ------------------"<<  bt_status_2 << std::endl;
        if(bt_status_1==0 && bt_last1){
         std::cout<< "Get button GREEN :: "<<  bt_status_2 << std::endl;
            car::MAX_SPD = car::MAX_SPD_GREEN;
            car::GAM_SPD = car::GAM_SPD_GREEN;
            car::DEL_SPD= car::DEL_SPD_GREEN;
        }
        bt_last1 = bt_status_1;

        if(bt_status_2==0 && bt_last2){

         std::cout<< "Get button RED ::  "<<  bt_status_1 << std::endl;
            car::MAX_SPD = car::MAX_SPD_RED;
            car::GAM_SPD = car::GAM_SPD_RED;
            car::DEL_SPD= car::DEL_SPD_RED;
        }
        bt_last2 = bt_status_2;

        if(count_ % 25 ==0){
            count_ = 0;
            if(summ <=3){
                SSENSOR = true;
            }
            else SSENSOR = false;
            summ = 0;
        }

        summ += sensor_status;
        count_++;

        key = getkey();

        if(key == '9'){
            stop();
            color.destroy();
            depth.destroy();
            openni::OpenNI::shutdown();

            ////////  Release //////////////////////////////////////////////////////
            if(conf::WRITE_VIDEO){
                color_videoWriter.release();
                depth_videoWriter.release();
                fclose(LogFile);
            }

            break;
        }

        if(stop_d ){
            status_break = true;
            sleep(3);
            run = true;
            start = car::MIN_SPD;
            process = true;
            stop_d= 0;
            run_mpu = false;
            checkPoint = false;
        }

        int speed = control(key, SSENSOR);

        if(run){
           cout << "speed " << speed << std::endl;
            api_set_FORWARD_control( pca9685,speed);
        } else {
            stop();
        }

        if(process){
            st = getTickCount();
            frame_id ++;
            frame_stop++;
            et = getTickCount();
            fps += 1.0 / ((et-st)/freq);
            cerr << "FPS -- : "<< fps/frame_id << " " << status::ANGLE << '\n';
        }
    }

    return 0;
}

