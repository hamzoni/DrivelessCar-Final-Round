#include "t.h"

VideoStream depth;
VideoStream color;
VideoFrameRef frame_color;
VideoFrameRef frame_depth;
cv::Mat depthImg = cv::Mat(240, 320, CV_16UC1);
cv::Mat colorImg = cv::Mat(480, 640, CV_8UC3);
cv::Mat rgb;
cv::Mat depthCam;
Status rc;
Device device;
VideoWriter color_videoWriter;
VideoWriter depth_videoWriter;

Mat get_depth(){
    depth.readFrame(&frame_depth);
//    analyzeDepth(frame_depth,  depthImg);
    depthImg = cv::Mat(240, 320, CV_16U);
    DepthPixel* depth_img_data;
    Mat depth_img_8u;
    depth_img_data = (DepthPixel*)frame_depth.getData();
    memcpy(depthImg.data, depth_img_data, 320*240*sizeof(DepthPixel));
    cv::threshold(depthImg,depthImg,25000,255,cv::THRESH_TOZERO_INV);
    normalize(depthImg, depth_img_8u, 255, 50, NORM_MINMAX);


    depth_img_8u.convertTo(depthImg, CV_8UC1);


    return depthImg;
}

Mat adjustDepth(const Mat& depth_img) {
    // from https://orbbec3d.com/product-astra/
    // Astra S has a depth in the range 0.35m to 2.5m
    //high velocity 18000->20000
    int maxDepth = 15000;
    int minDepth = 0; // in mm
    Mat res = depth_img;

    for (int j = 0; j < res.rows; j++)
        for (int i = 0; i < res.cols; i++) {
            if (res.at<ushort>(j, i)) {
                int tmp = res.at<ushort>(j, i);

                if (tmp < minDepth || tmp > maxDepth) res.at<ushort>(j, i) = 0;
                else
                    res.at<ushort>(j, i) = 255;

                // retImage.at<ushort>(j, i) = maxDepth - (retImage.at<ushort>(j, i) - minDepth);
            }
        }

    return res;
}


Mat get_color(){
    color.readFrame(&frame_color);
    //analyzeColor(frame_color, colorImg);
    RGB888Pixel* color_img_data;
    color_img_data = (RGB888Pixel*)frame_color.getData();
    memcpy(colorImg.data, color_img_data, 640*480*sizeof(RGB888Pixel));
    return colorImg;
}


void analyzeColor(const VideoFrameRef &frame_color, cv::Mat& color_img) {
    RGB888Pixel* color_img_data;
    color_img = Mat(HE,WI, CV_8UC3);
    color_img_data = (RGB888Pixel*)frame_color.getData();
    memcpy(color_img.data, color_img_data, HE*WI*sizeof(RGB888Pixel));
}

void analyzeDepth(const VideoFrameRef &frame_depth, cv::Mat& depth_img) {

    DepthPixel* depth_img_data;
//    depth_img = Mat(240, 320, CV_16U);
    Mat depth_img_8u;
    depth_img_data = (DepthPixel*)frame_depth.getData();
    memcpy(depth_img.data, depth_img_data, 320*240*sizeof(DepthPixel));
    normalize(depth_img, depth_img_8u, 255, 50, NORM_MINMAX);
    depth_img_8u.convertTo(depth_img, CV_8U);

}

int initCam(){

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
            depth_mode.setResolution(320,  240);
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
            color_mode.setFps(30);
            color_mode.setResolution(640, 480);
            color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            color.setVideoMode(color_mode);

            rc = color.start();
            if (rc != STATUS_OK){
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }
    std::cout << "----------------------------------------" << std::endl;

    VideoStream* streams[] = {&depth, &color};
    depthCam = get_depth();
    rgb = get_color();

    Size output_size_color(640, 480);
    Size output_size_depth(320, 240);

    if(conf::WRITE_VIDEO) {
        color_videoWriter.open("color.avi", CV_FOURCC('D','I','V', 'X'), 8, output_size_color, true);
        depth_videoWriter.open("depth.avi", CV_FOURCC('D','I','V', 'X'), 8, output_size_depth, false);
	}

}
