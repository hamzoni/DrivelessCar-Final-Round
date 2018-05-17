#include "api_kinect_cv.h"
#include "api_i2c_pwm.h"
#include <iostream>
#include "Hal.h"
#include <config.h>
#include <getparam.h>
#include "haar.h"
using namespace EmbeddedFramework;

using namespace openni;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640
#define VIDEO_FRAME_HEIGHT 480
PCA9685 *pca9685 = new PCA9685() ;
GPIO *gpio = new GPIO();
I2C *i2c_device = new I2C();
#define SENSOR	165
#define BUTTON	164
#define STARTX 20
#define LED		166

void analyzeFrameColor(const VideoFrameRef& frame_color, Mat& color_img) {
    RGB888Pixel* color_img_data;

    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    color_img = Mat(h, w, CV_8UC3);

    color_img_data = (RGB888Pixel*)frame_color.getData();
    memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));
	// flip(color_img, color_img, 1);
}

void analyzeFrameDepth(const VideoFrameRef& frame_depth, Mat& depth_img) {
    DepthPixel* depth_img_data;

    int w = frame_depth.getWidth();
    int h = frame_depth.getHeight();

    depth_img = Mat(h, w, CV_16U);
    // Mat depth_img_8u;

    // PIXEL_FORMAT_DEPTH_100_UM
    depth_img_data = (DepthPixel*)frame_depth.getData();

    memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));

    // flip(depth_img, depth_img, 1);

    // normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);
    // depth_img_8u.convertTo(depth_img_8u, CV_8U);
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


int main( int argc, char* argv[] ) {

    init("/home/ubuntu/Desktop/prototype");

    setup(pca9685);
    Status rc;
    Device device;
    int sensor = 0;
    int sw4_stat = 1;

	gpio->gpioExport(SENSOR);
	gpio->gpioExport(LED);
    gpio->gpioExport(BUTTON);
	gpio->gpioSetDirection(BUTTON, INPUT);
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
            depth_mode.setResolution(VIDEO_FRAME_WIDTH / 2, VIDEO_FRAME_HEIGHT / 2);
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

    VideoFrameRef  frame_color, frame_depth;
    VideoStream* streams[] = {&depth, &color};

    bool is_save_file = true; // set is_save_file = true if you want to log video and i2c pwm coeffs.
    VideoWriter color_videoWriter;

	string color_filename = "color.avi";

	Mat depthImg, colorImg;
	int codec = CV_FOURCC('D','I','V', 'X');
	int video_frame_width = VIDEO_FRAME_WIDTH;
    int video_frame_height = VIDEO_FRAME_HEIGHT;
	Size output_size(video_frame_width, video_frame_height);

	if(conf::WRITE_VIDEO) {
        color_videoWriter.open(color_filename, codec, 8, output_size, true);
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
    vector<Mat> frameVector(2);
    int zero= 0, one = 0, count=0, last_count= 0;
    bool run = true, btn4 = true, SSENSOR = false;
    unsigned int last = 0, sum = 0, last_sum = 1;
    int start = STARTX;
    int last_status = 0;
    unsigned int bt_status = 1;

    while (true)
    {
        st = getTickCount();
        key = getkey();
        unsigned int sensor_status = 0;
		gpio->gpioGetValue(SENSOR, &sensor_status);
		gpio->gpioGetValue(BUTTON, &bt_status);

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
            angle = 0;
            api_set_STEERING_control(pca9685 , angle);
            stop();
        }
        if(!SSENSOR && btn4){
            running = 1;
            throttle = 0;
            start = STARTX;
        }
        if( key == '2') {
            btn4 = true;
            start = STARTX;
            frame_id = 0;
            running = !running;
            throttle = 0;
            api_set_FORWARD_control( pca9685,throttle);
        } else if( key == '4') {
            running = !running;
            start = 0;
            frame_id= 0;
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

            frame_id ++;

            // Color
            color.readFrame(&frame_color);
            analyzeFrameColor(frame_color, colorImg);

            // Depth
            depth.readFrame(&frame_depth);
            analyzeFrameDepth(frame_depth, depthImg);


            if (conf::WRITE_VIDEO) {

                if (!colorImg.empty())
                    color_videoWriter.write(colorImg);

            }

            cv::Mat hsv, gray, g2;

            cv::cvtColor(colorImg, hsv, cv::COLOR_RGB2HSV);
            cv::cvtColor(colorImg, gray, cv::COLOR_RGB2GRAY);
            // cv::imshow("RBG", colorImg);
            int flag = detect_haar(colorImg);
            std::cout<< "turn ------  " << flag << std::endl;
            std::vector<double> angles;
            cv::imshow ("adf",colorImg);
            // speed
            if(start != throttle   && frame_id <= 100){
                throttle+=3;
                start++;
                frame_id++;
                throttle = start;
            }

            api_set_FORWARD_control( pca9685 , throttle);


            // angle
            api_set_STEERING_control(pca9685 , angle);

            et = getTickCount();
            fps = 1.0 / ((et-st)/freq);
            cerr << "FPS: "<< fps<< '\n';
            waitKey(conf::WAIT_KEY);
            if( key == 27 ) break;
        }
        else {
            if (!stopped) {
                stopped = true; started = false;
			}
        }
    }
    //////////  Release //////////////////////////////////////////////////////
	if(conf::WRITE_VIDEO)
    {
        // gray_videoWriter.release();
        color_videoWriter.release();
	}
    return 0;
}


