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
#include "objconf.cpp"

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
unsigned int bt_status_1 = 1, bt_status_2= 1, bt_status_3 = 1, bt_status_4 = 1;
bool checkPoint = false;
bool run_mpu = false;
int count_check_point = 0;
float mpu_value= 0, gold_value = 0;
int mpu_speed = 0;
long frame_id = 0;
void stop();
void brake_();
void setup(PCA9685 *pca9685);
int i =0;
int speed_checkPoint = 0;
bool detect_steel =  false;
int mpu_value_comp = car::MPU_LEFT;
bool save_left = false;
void thread_mpu(){
    while(1){
        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            //std::cout<< "mpu ----------" << imuData.fusionPose.z() *180/3.14 <<  std::endl;
            mpu_value = signhaar::haar.getDir(imuData.compass.y(), imuData.compass.x());
            //std::cout<< "mpu ----------" << imuData.fusionPose.z()*180/3.14 <<  std::endl;

            //std::cout << "---xz --------------" << getxz( (int)imuData.compass.x(),  (int)imuData.compass.z()) <<std::endl;
            //std::cout << signhaar::haar.getDir(imuData.compass.y(), imuData.compass.x()) << std::endl;
            //std::cout<< "mpu angle: ------------------------   "<< (int)imuData.compass.x() <<"   "<<  (int)imuData.compass.y()<< "  "<< (int)imuData.compass.z()<< std::endl;
            //std::cout<< "mpu value: -------" << mpu_value << std::endl;
            //cv::Mat mpuMat = colorImg.clone();
           // cv::imshow("mpu", mpuMat);
            //cv::waitKey(1);
//
            if(!save_left){
                mpu_value_comp = car::MPU_RIGHT;

            } else {

            mpu_value_comp = car::MPU_LEFT;
            }


            float mpu_y = imuData.compass.y();
            float mpu_z = imuData.fusionPose.z()*180/3.14;
           // bool left_m = ((((std::abs(mpu_value - MPU_left_angle)) <= 4) || (std::abs(mpu_z - MPU_left_comp)) <= 4)) && status::LEFT;

            if( std::abs(mpu_value - mpu_value_comp) <= 7  && run_mpu ){
                checkPoint = true;
            } else {
                checkPoint = false;
            }

            //std::cout << "che--------------------- << "<< std::abs(mpu_value - mpu_value_comp) << " " << mpu_value_comp << std::endl;
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
            // cv::waitKey(1);

        } catch (exception &e){}
    }
}

void show_sign(){
    while(1){
        try{
            frameid++;
            cv::Mat signImg, img_con;
            rgb.copyTo(signImg);
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
                status::LEFT = false;
                std::cout << "Turn Right" << std::endl;
            }
            if (id == 2) {
                status::LEFT = true;
                std::cout << "Turn Left" << std::endl;
            }
           //  cv::imshow("signs", img_con);
           // cv::waitKey(1);
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
            status::OBJ = objdetect::detector.get_object_depth(depthCam, img_lane, object_rect);
            //cout<< "Fast Check object : "<<status::OBJ<<"\n";

           // updateObj(object_rect, 2, 2);
            cv::resize(img_lane, img_lane, cv::Size(conf::WIDTH, conf::HEIGHT));
            processImg(img_lane);
//            processImgSe(img_lane);
            //std::cout<< "angle: " << status::ANGLE << std::endl;
            if(status::NGABA ){
                save_left = status::LEFT;
            }
            api_set_STEERING_control(pca9685 , status::ANGLE);
        } catch (exception &e){}
    }
}

bool psensor = false;
bool run_stop = false;
bool call_brake = false;
int count_brake = 0;

int control(char key, bool sensor){

    if(sensor == false && psensor){
        key = '2';
        start = car::MIN_SPD;
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
        run_mpu = false;
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
    speed = status::SPEED;

    if(dis_stop == 2){
        run_mpu = true;
    }

    if(checkPoint){
        run = false;
        // stop();
        brake_();
        run_mpu = false;
        //detect_steel = true;
        stop_d = 1;
        //speed_checkPoint = 20;
        call_brake = true;
    }

    return speed;
}

void thread_read_color(){
    while(1){
        cv::Mat imgColor = get_color();
        if (conf::WRITE_VIDEO) {
            if (!imgColor.empty()){
                        cv::putText(colorImg,  std::to_string((int) mpu_value),cv::Point(50,50), 3, 1, cv::Scalar(0,0,255),4);
                color_videoWriter.write(imgColor);}
        }
        imgColor.copyTo(rgb);

        //cv::imshow("color_", imgColor);
        //cv::waitKey();
    }
}

void thread_read_depth(){
    while(1){
        cv::Mat imgDepth = get_depth();

        if (conf::WRITE_VIDEO) {
            if (!imgDepth.empty())
                depth_videoWriter.write(imgDepth);
        }
        //imgDepth = adjustDepth(imgDepth);
        //depthImg.convertTo(depthImg, CV_8UC1, 255);
        //cv::imshow("depth", imgDepth);
        //cv::waitKey(1);
        imgDepth.copyTo(depthCam);
    }
}

void thread_main(){

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
    mpu_value_comp = car::MPU_LEFT;
    setup(pca9685);

    FILE *LogFile = fopen("thetaLog.txt", "w");
	gpio->gpioExport(SENSOR);
	gpio->gpioExport(LED);
    gpio->gpioExport(BUTTON1);
    gpio->gpioExport(BUTTON2);
    gpio->gpioExport(BUTTON3);
    gpio->gpioExport(BUTTON4);
	gpio->gpioSetDirection(BUTTON1, INPUT);
    gpio->gpioSetDirection(BUTTON2, INPUT);
	gpio->gpioSetDirection(BUTTON3, INPUT);
    gpio->gpioSetDirection(BUTTON4, INPUT);

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
    unsigned int last = 0, sum = 0, bt_last1 = 1, bt_last2 = 1, bt_last3 = 1,bt_last4 = 1 ;

    std::thread cam_color(&thread_read_color);
    //std::thread cam_depth(&thread_read_depth);
    std::thread lane_(&show_lane);
    std::thread sign_(&show_sign);
    std::thread stop_(&show_stop);

    //std::thread stop_(&thread_stop);
    std::thread mpu_(&thread_mpu);
    cam_color.detach();
    //cam_depth.detach();
    mpu_.detach();
    lane_.detach();
    sign_.detach();
    stop_.detach();

    while (true){
        frame_id++;
        unsigned int sensor_status = 0;
		gpio->gpioGetValue(SENSOR, &sensor_status);
		gpio->gpioGetValue(BUTTON1, &bt_status_1);
        gpio->gpioGetValue(BUTTON2, &bt_status_2);
        gpio->gpioGetValue(BUTTON3, &bt_status_3);
        gpio->gpioGetValue(BUTTON4, &bt_status_4);
		//std::cout<< "button1 :: ------------------"<<  bt_status_1 << std::endl;
        //std::cout<< "button2 :: ------------------"<<  bt_status_2 << std::endl;
        if(bt_status_1==0 && bt_last1){
         std::cout<< "Get button GREEN -------------:: "<<  bt_status_2 << std::endl;
            car::MAX_SPD = car::MAX_SPD_GREEN;
            car::GAM_SPD = car::GAM_SPD_GREEN;
            car::DEL_SPD= car::DEL_SPD_GREEN;
            start = car::MIN_SPD;
            key = '2';
        }
        bt_last1 = bt_status_1;

        if(bt_status_2==0 && bt_last2){

         std::cout<< "Get button RED --------------::  "<<  bt_status_1 << std::endl;
            car::MAX_SPD = car::MAX_SPD_RED;
            car::GAM_SPD = car::GAM_SPD_RED;
            car::DEL_SPD= car::DEL_SPD_RED;
            start = car::MIN_SPD;
            key = '2';
        }
        bt_last2 = bt_status_2;

        if(bt_status_3==0 && bt_last3){
         std::cout<< "Get button GREEN UP --------------:: "<<  bt_status_3 << std::endl;
         obj::TYPE= 3;

        }
        bt_last1 = bt_status_1;

        if(bt_status_4==0 && bt_last4){
         std::cout<< "Get button RED UP ------------:: "<<  bt_status_4 << std::endl;
        obj::TYPE = 2;
        }
        bt_last4 = bt_status_4;

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

void brake_(){
    usleep(car::MICRO_SECOND);
    int throttle_stop = 0;
    api_set_FORWARD_control( pca9685,throttle_stop);
    int ss = car::BACK_SPD;
    api_set_REVERSE_control( pca9685, ss);
    run = true;
    start = car::MIN_SPD;
    process = true;
    stop_d= 0;
    run_mpu = false;
    checkPoint = false;
    detect_steel = false;
    speed_checkPoint = 0;
    usleep(5500000);
}
void stop(){
    int zezo = 0;
    api_set_FORWARD_control( pca9685,zezo);

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

