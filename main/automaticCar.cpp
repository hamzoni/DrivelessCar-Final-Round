#include "api_kinect_cv.h"
#include "api_i2c_pwm.h"
#include <iostream>
#include "Hal.h"
using namespace openni;
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms
#define VIDEO_FRAME_WIDTH 640	
#define VIDEO_FRAME_HEIGHT 480

#define SW1_PIN	160
#define SW2_PIN	161
#define SW3_PIN	163
#define SW4_PIN	164
#define SENSOR	165

char analyzeFrame(const VideoFrameRef& frame_depth,const VideoFrameRef& frame_color,Mat& depth_img, Mat& color_img) {
    DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;

    int w = frame_color.getWidth();
    int h = frame_color.getHeight();

    depth_img = Mat(h, w, CV_16U);
    color_img = Mat(h, w, CV_8UC3);
    Mat depth_img_8u;
	

    depth_img_data = (DepthPixel*)frame_depth.getData();

    memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));

    normalize(depth_img, depth_img_8u, 255, 0, NORM_MINMAX);

    depth_img_8u.convertTo(depth_img_8u, CV_8U);
    color_img_data = (RGB888Pixel*)frame_color.getData();

    memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));

    cvtColor(color_img, color_img, COLOR_RGB2BGR);

    return 'c';
}


int main( int argc, char* argv[] ) {

    Status rc;
    Device device;

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
    VideoWriter gray_videoWriter;
     
    string gray_filename = "gray.avi";
	string color_filename = "color.avi";
	string depth_filename = "depth.avi";
	
	Mat depthImg, colorImg, grayImage;
	int codec = CV_FOURCC('D','I','V', 'X');
	int video_frame_width = VIDEO_FRAME_WIDTH;
    int video_frame_height = VIDEO_FRAME_HEIGHT;
	Size output_size(video_frame_width, video_frame_height);

   	FILE *thetaLogFile; // File creates log of signal send to pwm control
	if(is_save_file) {
	    gray_videoWriter.open(gray_filename, codec, 8, output_size, false);
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


    bool is_show_cam = true;
	int count_s,count_ss;
    int frame_id = 0;
	vector<cv::Vec4i> lines;
    while ( true )
    { 

        st = getTickCount();
        key = getkey();


		
        if( key == 's') {
            running = !running;
        }
        if( key == 'f') {
            fprintf(stderr, "End process.\n");
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
		char recordStatus = analyzeFrame(frame_depth,frame_color, depthImg, colorImg);
		flip(depthImg, depthImg, 1);
		flip(colorImg, colorImg, 1);
		
            ////////// Detect Center Point ////////////////////////////////////
            if (recordStatus == 'c') {
                cvtColor(colorImg, grayImage, CV_BGR2GRAY);
                cv::imshow("two", colorImg);
            }

            et = getTickCount();
            fps = 1.0 / ((et-st)/freq);
            cerr << "FPS: "<< fps<< '\n';

            if (recordStatus == 'c' && is_save_file) {
                // 'Center': target point
                // pwm2: STEERING coefficient that pwm at channel 2 (our steering wheel's channel)
                
                if (!colorImg.empty())
			        color_videoWriter.write(colorImg);
			    if (!grayImage.empty())
			        gray_videoWriter.write(grayImage); 
            }
            if (recordStatus == 'd' && is_save_file) {
                if (!depthImg.empty())
                   depth_videoWriter.write(depthImg);
            }

            //////// using for debug stuff  ///////////////////////////////////
            if(is_show_cam) {
                if(!grayImage.empty())
                    imshow( "gray", grayImage );
                waitKey(10);
            }
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
	if(is_save_file)
    {
        gray_videoWriter.release();
        color_videoWriter.release();
        //depth_videoWriter.release();
        fclose(thetaLogFile);
	}
    return 0;
}


