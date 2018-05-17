//
// Created by nc on 29/04/2018.
//

#include <config.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "greenimageprocess.h"
#include "greconf.h"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "laneimageprocess.h"

void findContoursThres(cv::Mat &src,
                       std::vector<std::vector<cv::Point>> &contours) {
    cv::Mat ch[3], merged;
    split(src, ch);
    merged = ch[2];

    for (auto &kernel : gre::kernels_morp) {
        cv::morphologyEx(merged, merged, CV_MOP_CLOSE, kernel);
        cv::morphologyEx(merged, merged, CV_MOP_OPEN, kernel);
    }

    morphologyEx(merged, merged, CV_MOP_GRADIENT, gre::kernel_gradient);

    cv::GaussianBlur(merged,merged, cv::Size(3,3), 0 ,0);
    cv::threshold(merged, merged, 0, 255, cv::THRESH_OTSU | cv::THRESH_BINARY);

    findContours(merged, contours, CV_RETR_CCOMP , cv::CHAIN_APPROX_SIMPLE);
}

void findContoursSobel(cv::Mat &src,
                       std::vector<std::vector<cv::Point>> &contours) {
    cv::Mat channel[3];
    cv::split(src, channel);

    cv::Mat grad_x, grad_y, grad;
    cv::Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    cv::Scharr(channel[2], grad_x, CV_16S, 1, 0, gre::SCALE_SCHARR, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    cv::Scharr( channel[2], grad_y, CV_16S, 0, 1, gre::SCALE_SCHARR, 0, cv::BORDER_DEFAULT );
    cv::convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
    cv::GaussianBlur( grad, grad, cv::Size(5,5), 1, 1, cv::BORDER_DEFAULT );
    cv::threshold(grad , grad, gre::THRES, 1, cv::THRESH_OTSU);


    findContours(grad, contours, cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE);
}

void findContoursNormColor(cv::Mat &src,
                           std::vector<std::vector<cv::Point>> &contours) {
    unsigned char rangeH, rangeS;
    /*============DEFINE RANGE FOR HUE COLOR==============*/
    int l_green;
    /*Upper thresh*/
    int h_green;
    /*====================================================*/
    /*Specific range*/
    l_green = gre::L_GREEN;
    h_green = gre::H_GREEN;

    cv::Mat gray = cv::Mat::zeros(src.size(), CV_8UC1);

    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            if (src.at<cv::Vec3b>(y, x)[0] >= l_green && src.at<cv::Vec3b>(y, x)[0] < h_green) {
                int threshG = gre::THRES;
                if (src.at<cv::Vec3b>(y, x)[1] < threshG && src.at<cv::Vec3b>(y, x)[2] >= threshG) {
                    //White case
                    rangeS = 0;
                } else if (src.at<cv::Vec3b>(y, x)[1] >= threshG && src.at<cv::Vec3b>(y, x)[2] >= threshG) {
                    //Light Color case
                    rangeS = 255;
                } else if (src.at<cv::Vec3b>(y, x)[1] >= threshG && src.at<cv::Vec3b>(y, x)[2] < threshG) {
                    //Dark Color case
                    rangeS = 255;
                } else {
                    //Black case
                    rangeS = 0;
                }
                gray.at<unsigned char>(y, x) = rangeS;
            }   else {
                gray.at<unsigned char>(y, x) = rangeS;
            }
        }
    }


    for (auto &kernel : gre::kernels_morp) {
        cv::morphologyEx(gray, gray, CV_MOP_CLOSE, kernel);
        cv::morphologyEx(gray, gray, CV_MOP_CLOSE, kernel);
    }

    cv::GaussianBlur(gray,gray, cv::Size(3,3), 0 ,0);
    cv::threshold(gray, gray, gre::THRES, 255, cv::THRESH_BINARY);
    cv::findContours(gray, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    cv::imshow("adf", gray);

}





void findContoursInRangeColor(cv::Mat &src,
                              std::vector<std::vector<cv::Point>> &contours) {
    cv::Mat mask;
    cv::inRange(src, gre::lowHSV, gre::highHSV, mask);
    cv::medianBlur(mask, mask, 5);
    cv::findContours(mask, contours, cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE);
}

void findContoursNormColorHarCode(cv::Mat &src,
                                  std::vector<std::vector<cv::Point>> &contours) {
    int rangeH, rangeS, rangeV;
    /*============DEFINE RANGE FOR HUE COLOR==============*/
    int max_hue = 179;
    int min_hue = 0;
    /*Lower thresh*/
    int l_yellow, l_green, l_blue, l_magneta, l_red;
    /*Upper thresh*/
    int h_yellow, h_green, h_blue, h_magneta, h_red;
    /*====================================================*/

    /*Specific range*/
    l_green = gre::L_GREEN;
    h_green = gre::H_GREEN;

    l_blue = 97;
    h_blue = 118;

    l_red = 169;
    h_red = 8;
    /*------------*/

    cv::Mat dst = src.clone();
    for (int y = 0; y < dst.rows; ++y) {
        for (int x = 0; x < dst.cols; ++x) {
            /*===================HUE PROCESSING==========================*/
            if (dst.at<cv::Vec3b>(y, x)[0] >= l_green && dst.at<cv::Vec3b>(y, x)[0] < h_green) {
                /*===========================GREEN================================*/
                rangeH = 57;
                /*===================SATURATION + VALUE PROCESSING================*/
                int threshG = gre::THRES;
                if (dst.at<cv::Vec3b>(y, x)[1] < threshG && dst.at<cv::Vec3b>(y, x)[2] >= threshG) {
                    //White case
                    rangeS = 0;
                    rangeV = 255;
                } else if (dst.at<cv::Vec3b>(y, x)[1] >= threshG && dst.at<cv::Vec3b>(y, x)[2] >= threshG) {
                    //Light Color case
                    rangeS = 255;
                    rangeV = 255;
                } else if (dst.at<cv::Vec3b>(y, x)[1] >= threshG && dst.at<cv::Vec3b>(y, x)[2] < threshG) {
                    //Dark Color case
                    rangeS = 255;
                    rangeV = 255;
                } else {
                    //Black case
                    rangeS = 0;
                    rangeV = 255;
                }
                dst.at<cv::Vec3b>(y, x)[1] = rangeS;
                dst.at<cv::Vec3b>(y, x)[2] = rangeV;
                /*=================================================================*/
            }   else {
                dst.at<cv::Vec3b>(y, x)[0] = 0;
                dst.at<cv::Vec3b>(y, x)[1] = 0;
                dst.at<cv::Vec3b>(y, x)[2] = 255;
            }
            dst.at<cv::Vec3b>(y, x)[0] = rangeH;
        }
    }

    cv::imshow("hardcode", dst);
    cv::Mat gray;
    cv::Mat hsv_channels[3];
    cv::split( dst, hsv_channels);
    gray = hsv_channels[1];

    for (auto &kernel : gre::kernels_morp) {
        cv::morphologyEx(gray, gray, CV_MOP_CLOSE, kernel);
    }
    cv::GaussianBlur(gray,gray, cv::Size(3,3), 0 ,0);
    cv::findContours(gray, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
}

void findContoursThresWhite(cv::Mat &src,
                            std::vector<std::vector<cv::Point>> &contours){
    findContours(src, contours, cv::RETR_TREE , cv::CHAIN_APPROX_SIMPLE);
}


void getMaskThres(cv::Mat &src,
                  cv::Mat &dst) {
    std::vector<std::vector<cv::Point>> contours;
    findContoursThres(src, contours);

    dst = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point> > approx(contours.size());

    for(int i = 0; i < contours.size(); i++ ) {

        int area_cnt = cv::contourArea(contours[i]);

        if(area_cnt >= gre::MIN_AREA_GREEN_CNT){
            std::vector<cv::Point> tmp = contours.at(i);
            const cv::Point* elementPoints[1] = { &tmp[0] };
            int numberOfPoints = (int)tmp.size();
            cv::fillPoly (dst, elementPoints, &numberOfPoints, 1, cv::Scalar (255, 255, 255), 8);
        }
    }
}

void getMaskSobel(cv::Mat &src,
                  cv::Mat &dst) {
    std::vector<std::vector<cv::Point>> contours;
    findContoursSobel(src, contours);
    dst = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point> > hull(contours.size());

    for( int i = 0; i< contours.size(); i++ )
    {
        int area_cnt = cv::contourArea(contours[i]);
        if(area_cnt >= gre::MIN_AREA_GREEN_CNT){
            drawContours(dst, contours, i, gre::white, 2, 2);
        }
    }

//    mask2.copyTo(dst, gre::mask_roi);
}

void getMaskNormColor(cv::Mat &src,
                      cv::Mat &dst) {
    std::vector<std::vector<cv::Point>> contours;
    findContoursNormColor(src, contours);

    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    find_if_contours_cloes(contours);
    std::vector<std::vector<cv::Point>> hull(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        cv::convexHull( cv::Mat(contours[i]), hull[i], false );
    }

    drawContours(dst, hull, -1, 255, 2);
    cv::rectangle(dst, gre::roi_mask, cv::Scalar(0), 10);
//    cv::imshow("green", dst);

}

void getMaskInRangeColor(cv::Mat &src,
                         cv::Mat &dst) {

    std::vector<std::vector<cv::Point>> contours;
    findContoursInRangeColor(src, contours);

    cv::Mat mask2 = cv::Mat::zeros(src.size(), CV_8UC1);

//    std::vector<std::vector<cv::Point> > approx(contours.size());
    std::vector<std::vector<cv::Point> > hull(contours.size());
    for( int i = 0; i< contours.size(); i++ )
    {
        int area_cnt = cv::contourArea(contours[i]);
        if(area_cnt >= gre::MIN_AREA_GREEN_CNT){
//            cv::approxPolyDP(cv::Mat(contours[i]), approx[i],3, true);
            cv::convexHull( cv::Mat(contours[i]), hull[i], false );
            if(area_cnt/contourArea(hull[i]) >= gre::RATIO_CNT_CONVEXHULL){
                drawContours(mask2, hull  , i, gre::white, 3, 1);

            }
        }

    }
    dst = mask2;
//    dst = cv::Mat(mask2.rows, mask2.cols, mask2.type(), cv::Scalar::all(0));
//    mask2.copyTo(dst, gre::mask_roi);
}

void getMaskNormColorHardCode(cv::Mat &src,
                              cv::Mat &dst){
    std::vector<std::vector<cv::Point>> contours;
    findContoursNormColorHarCode(src, contours);

    dst = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point> > hull;
    for (int i = 0; i < contours.size(); i++) {
        if(cv::contourArea(contours[i]) >= gre::MIN_AREA_GREEN_CNT){
            std::vector<cv::Point> sub_hull;
            cv::convexHull( cv::Mat(contours[i]), sub_hull, false );
            hull.push_back(sub_hull);
        }
    }

//    find_if_contours_cloes(hull);

    drawContours(dst, hull, -1, 255, 2);
}

void FindBlobs(const cv::Mat &binary,
               std::vector < std::vector<cv::Point2i>> &blobs) {
    blobs.clear();

    // Fill the label_image with the blobs
    // 0  - background
    // 1  - unlabelled foreground
    // 2+ - labelled foreground

    cv::Mat label_image;
    binary.convertTo(label_image, CV_32SC1);

    int label_count =2; // starts at 2 because 0,1 are used already

    for(int y=0; y < label_image.rows; y++) {
        int *row = (int*)label_image.ptr(y);
        for(int x=0; x < label_image.cols; x++) {
            if(row[x] != 1) {
                continue;
            }

            cv::Rect rect;
            cv::floodFill(label_image, cv::Point(x,y), label_count, &rect, 0, 0, 4);

            std::vector <cv::Point2i> blob;

            for(int i=rect.y; i < (rect.y+rect.height); i++) {
                int *row2 = (int*)label_image.ptr(i);
                for(int j=rect.x; j < (rect.x+rect.width); j++) {
                    if(row2[j] != label_count) {
                        continue;
                    }

                    blob.push_back(cv::Point2i(j,i));
                }
            }

            blobs.push_back(blob);

            label_count++;
        }
    }
}

bool find_if_contours_cloes(std::vector<std::vector<cv::Point>> &contours){
    if(contours.size() > 1){
        cv::Rect r1, r2, r_inter;
        int dis_two_rects, d1, d2, smaller_area, greater_area;

        for(int i = 0; i < contours.size() -1; i++){
            r1 = cv::boundingRect(contours[i]);
            for(int j = i +1; j < contours.size(); j++){
                r2 = cv::boundingRect(contours[j]);
                smaller_area = r1.area();
                greater_area = r2.area();

                if(smaller_area > greater_area){
                    int swap = smaller_area;
                    smaller_area = greater_area;
                    greater_area = swap;
                }

                double ratio = greater_area / smaller_area;

//                if(ratio <= 2){
//                    if(r1.x < r2.x){
//                        d1 = std::abs(r1.x + r1.width - r2.x) + std::abs(r1.y - r2.y);
//                        d2 = std::abs(r1.x + r1.width - r2.x) + std::abs(r1.y + r1.height - r2.y -r2.height);
//                    } else {
//                        d1 = std::abs(r2.x + r2.width - r1.x) + std::abs(r2.y - r1.y);
//                        d2 = std::abs(r2.x + r2.width - r1.x) + std::abs(r2.y + r2.height - r1.y -r1.height);
//                    }
//
//                    if(d1 + d2 < 40){
//                        contours[i].insert(contours[i].end(), contours[j].begin(), contours[j].end());
//                        contours.erase(contours.begin() + j);
//                        j--;
//                    } else {
                         if(r_inter.area() == smaller_area ){
                            contours[i].insert(contours[i].end(), contours[j].begin(), contours[j].end());
                            contours.erase(contours.begin() + j);
                            j--;
                        }


//                    }
//                }

//                std::cout << "ratio " << r1.area() << " " << r2.area() << " " << ratio << std::endl;
//                std::cout << "ratio " << d1 << " " << d2 << " " << ratio << std::endl;

            }
        }
    }
}

void findBolbs(cv::Mat &src, cv::Mat &output){
    std::vector < std::vector<cv::Point2i > > blobs;

    cv::Mat gray, binary;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, gre::THRES, 1, cv::THRESH_BINARY);

    output = cv::Mat::zeros(binary.size(), CV_8UC3);
    FindBlobs(gray , blobs);

    // Randomy color the blobs
    for(size_t i=0; i < blobs.size(); i++) {
        unsigned char r = 255 * (rand()/(1.0 + RAND_MAX));
        unsigned char g = 255 * (rand()/(1.0 + RAND_MAX));
        unsigned char b = 255 * (rand()/(1.0 + RAND_MAX));

        for(size_t j=0; j < blobs[i].size(); j++) {
            int x = blobs[i][j].x;
            int y = blobs[i][j].y;

            output.at<cv::Vec3b>(y,x)[0] = b;
            output.at<cv::Vec3b>(y,x)[1] = g;
            output.at<cv::Vec3b>(y,x)[2] = r;
        }
    }
}
