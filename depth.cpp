#include "api_kinect_cv.h"
#include "api_i2c_pwm.h"
#include <iostream>
#include "Hal.h"
#include <config.h>
#include <getparam.h>
#include <frameprocess.h>
#include "traffic.h"
#include <thread>
#include <mutex>
#include <carconf.h>
#include <fuzzylogic.h>
#include <realangle.h>
#include <detectorobj.h>
// 484 right
using namespace EmbeddedFramework;
using namespace openni;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640	
#define VIDEO_FRAME_HEIGHT 480
#define VIDEO_FRAME_MID 320
#define FLOOR 1
PCA9685 *pca9685 = new PCA9685() ;
GPIO *gpio = new GPIO();
I2C *i2c_device = new I2C();
std::mutex mu;
Traffic *d = new Traffic();
#define SENSOR	165
#define LED		166
vector<cv::Point> analyzeFrame (const VideoFrameRef& frame_depth,const VideoFrameRef& frame_color,Mat& depth_img, Mat& color_img) {
    DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;
    vector<Point> object(2);
    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    depth_img = Mat(h, w, CV_16U);
    color_img = Mat(h, w, CV_8UC3);
    Mat depth_img_8u;
    double floor[11] = {3, 2.1, 2.1, 2.5,1.3, 1.3, 0.9, 0.5, 0.13, 0.13 , 0.5};
            depth_img_data = (DepthPixel*)frame_depth.getData();
            memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));
            normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
            depth_img_8u.convertTo(depth_img_8u, CV_8U);
            color_img_data = (RGB888Pixel*)frame_color.getData();
            memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));
            cvtColor(color_img, color_img, COLOR_RGB2BGR);
            imshow("gfddgf", depth_img_8u);
            // imshow("asdf", color_img);
            depth_img =  depth_img_8u.clone();
            flip(color_img, color_img, 1);
            // for(int y= 0; y< VIDEO_FRAME_HEIGHT -1; y++){
                int y = 280; 
                int value = 12345;
                bool check = false;
                for(int x= 280;x < 380 ; x+=10){
                    int idx = x + y * VIDEO_FRAME_WIDTH  ;
                    const DepthPixel& rDepth = depth_img_data[idx];
                    double dis = std::tan(rDepth/1024.0 + 0.5)* 33.825 + 5.7;
                    // double dis = 38400.0/(1091.5 - rDepth);
                    for(int k = 0; k< 11; k++){
                    if(abs(dis - floor[k]) <= 0.5){
                            circle(color_img, cv::Point(x, y), 5, cv::Scalar(255,0,0), 3, 8, 0);
                        // cout<<"point 1\n"; 2.54
                    }
                    }

                    // if(abs(dis - left) >= 2) {
                    //     object[1] = cv::Point(x, y);
                    //     // cout<<"point2\n";
                    //     break;
                    // }    
                    

                    if(x == 330 )
                    cout<< "--------------------"<< dis << endl;
                }
            return object;
}

int stop(){
    int throttle_stop = 0;
    api_set_FORWARD_control( pca9685,throttle_stop);
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

void detec_thread(vector<Mat> frameVec){
    Mat im = frameVec.at(0).clone();
    int id = d->taquy(im);
    // int id = 0;
    if (id == 0) {
        conf::LEFT = 1;
    } else if (id == 1) {
        conf::LEFT = 0;
    }
    cout << id << endl;
}

int main( int argc, char* argv[] ) {

    init("/home/ubuntu/Desktop/prototype");
    std::cout << "Inserting filter...." << std::endl;
    objdetect::detector.insert_filter(cv::Scalar(121, 97, 70), cv::Scalar(131, 200, 254));

    setup(pca9685);
    Status rc;
    Device device;
    int sensor = 0;
	gpio->gpioExport(SENSOR);
	gpio->gpioExport(LED);
	gpio->gpioSetDirection(SENSOR, INPUT);
	gpio->gpioSetDirection(LED, OUTPUT);
    i2c_device->m_i2c_bus = 1;

    if (!i2c_device->HALOpen()) {
		printf("Cannot open I2C peripheral\n");
		exit(-1);
	} else printf("I2C peripheral is opened\n");
    VideoStream depth, color;
    rc = OpenNI::initialize();
    if (rc != STATUS_OK) {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 0;
    }
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK) {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 0;
    }
    if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc == STATUS_OK) {
            VideoMode depth_mode = depth.getVideoMode();
            depth_mode.setFps(30);
            depth_mode.setResolution(VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
            depth_mode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
            depth.setVideoMode(depth_mode);

            rc = depth.start();
            if (rc != STATUS_OK) {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        }
    }

    if (device.getSensorInfo(SENSOR_COLOR) != NULL) {
        rc = color.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK) {
            VideoMode color_mode = color.getVideoMode();
            color_mode.setResolution(VIDEO_FRAME_WIDTH, VIDEO_FRAME_HEIGHT);
            color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            color.setVideoMode(color_mode);

            rc = color.start();
            if (rc != STATUS_OK)
            {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    
    VideoFrameRef frame_depth, frame_color;
    VideoStream* streams[] = {&depth, &color};
    
    bool is_save_file = true; // set is_save_file = true if you want to log video and i2c pwm coeffs.
    VideoWriter depth_videoWriter;	
    VideoWriter color_videoWriter;
     
    string gray_filename = "gray.avi";
	string color_filename = "color.avi";
	string depth_filename = "depth.avi";
	
	Mat depthImg, colorImg;
	int codec = CV_FOURCC('D','I','V', 'X');
	int video_frame_width = VIDEO_FRAME_WIDTH;
    int video_frame_height = VIDEO_FRAME_HEIGHT;
	Size output_size(video_frame_width, video_frame_height);

   	FILE *thetaLogFile; // File creates log of signal send to pwm control
	if(conf::WRITE_VIDEO) {
	    depth_videoWriter.open(depth_filename, codec, 8, output_size, false);
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
        //depth_videoWriter.open(depth_filename, codec, 8, output_size, false);
        thetaLogFile = fopen("thetaLog.txt", "w");
	}
/// End of init logs phase ///

    int dir = 0, throttle_val = 0;
    double theta = 0;
    int current_state = 0;
    char key = 0;

    // Argc == 2 eg ./test-autocar 27 means initial throttle is 27
    int frame_width = VIDEO_FRAME_WIDTH;
    int frame_height = VIDEO_FRAME_HEIGHT;

    bool running = false, started = false, stopped = false;

    double st = 0, et = 0, fps = 0;
    double freq = getTickFrequency();
    double angle;

    int throttle = 0;

    bool is_show_cam = true;
    long frame_id = 0;
	vector<cv::Vec4i> lines;
    vector<Mat> frameVector(1);
    int zero= 0, one = 0, count=0, last_count= 0;
    bool run = true, btn4 = true, SSENSOR = false;
     unsigned int last = 0, sum = 0, last_sum = 1;
     const DepthPixel * pDepthArray = NULL;
    while (true)
    {
        st = getTickCount();
        key = getkey();
        unsigned int sensor_status = 0;
		gpio->gpioGetValue(SENSOR, &sensor_status);
        if(count %20==0){
            count = 0;
            if(sum <=3){
                SSENSOR = true;
            }
            else SSENSOR = false;
            sum = 0;
        }
        sum += sensor_status;
        count++;
        if(SSENSOR){
            running = 0;
            stop();
        }
        if(!SSENSOR && btn4){
            running = 1;
        }


        if( key == '2') {
            btn4 = true;
            running = !running;
            throttle = 0;
            api_set_FORWARD_control( pca9685,throttle);
        } else if( key == '4') {
            running = !running;
            stop();
            btn4 = false;
            
        } else if( key == '8') {
            stop();
            break;
        }
        
        if( running )
        {
            if (!started)
			{
    			fprintf(stderr, "ON\n");
			    started = true; stopped = false;
			}
            int readyStream = -1;
		    rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
		    if (rc != STATUS_OK)
		    {
		        printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
		        break;
		    }
            depth.readFrame(&frame_depth);
            color.readFrame(&frame_color);
            frame_id ++;
            vector<cv::Point>  depthObject = analyzeFrame(frame_depth,frame_color, depthImg, colorImg);
            // pDepthArray = (const DepthPixel*)frame_depth.getData();
            // int idx = 320 + 240 * VIDEO_FRAME_WIDTH ;
            //         const DepthPixel& rDepth = pDepthArray[idx];
            //         double dis = std::tan(rDepth/2047 + 0.5)* 33.825 + 5.7;
            //         if( )
            //           cout<< "depth --------------------    "<< dis << endl;
            // float fx, fy, fz;
            cout<< depthObject[0] << endl;
            // if (recordStatus == 'c') {
                line(colorImg,cv::Point(280, 280),cv::Point(390, 280), cv::Scalar(255,0,0), 3 ,8, 0 );
                line(colorImg, depthObject[0],depthObject[1], cv::Scalar(255,0,0), 3 ,8, 0 );
                // imshow("asdf", colorImg);
                if ( conf::WRITE_VIDEO) {
                // 'Center': target point
                // pwm2: STEERING coefficient that pwm at channel 2 (our steering wheel's channel)
                
                if (!colorImg.empty())
                    color_videoWriter.write(colorImg.clone());
                if (!depthImg.empty())
                   depth_videoWriter.write(depthImg); 
                }

                frameVector[0] = Mat();
                colorImg.copyTo(frameVector[0]);
                std::thread a(&detec_thread, frameVector);
                a.detach();if(a.joinable()) a.join();
                
                // imshow("asdfasdf", depthImg);
                // std::vector<double> angles; 
                // processImg(colorImg, angles);

  
                // speed
                // throttle = fuzzy(angles[1], conf::NGABA, conf::OBJ);  
                api_set_FORWARD_control( pca9685 , throttle);

                // angle
                // alluse(angles[0]);
                // angle = angles[0];
                // std::cout << angle << " -- " <<angles[1] << " ---- "  << throttle <<std::endl;
                api_set_STEERING_control(pca9685 , angle);

                if (key == 's') {
                    cv::imwrite("capture.jpg", colorImg);
                }
            // }
            et = getTickCount();
            fps = 1.0 / ((et-st)/freq);
            // cerr << "FPS: "<< fps<< '\n';

                     
            waitKey(conf::WAIT_KEY);
            if( key == 27 ) break;
        }
        else {
            if (!stopped) {
                fprintf(stderr, "OFF\n");
                stopped = true; started = false;
			}
        }
    }
    //////////  Release //////////////////////////////////////////////////////
	if(conf::WRITE_VIDEO)
    {
        // gray_videoWriter.release();
        color_videoWriter.release();
        depth_videoWriter.release();
        fclose(thetaLogFile);
	}
    return 0;
}


