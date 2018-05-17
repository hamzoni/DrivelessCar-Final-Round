//
// Created by nc on 29/04/2018.
//

#ifndef X_CDS_GREENIMAGEPROCESS_H
#define X_CDS_GREENIMAGEPROCESS_H

#include "opencv2/highgui/highgui.hpp"

void findContoursThres(cv::Mat &src,
                       std::vector<std::vector<cv::Point>> &contours);

void findContoursSobel(cv::Mat &src,
                       std::vector<std::vector<cv::Point>> &contours);

void findContoursNormColor(cv::Mat &src,
                           std::vector<std::vector<cv::Point>> &contours);

void findContoursInRangeColor(cv::Mat &src,
                              std::vector<std::vector<cv::Point>> &contours);

void findContoursNormColorHarCode(cv::Mat &src,
                                  std::vector<std::vector<cv::Point>> &contours);

void findContoursBinh(cv::Mat &src,
                                  std::vector<std::vector<cv::Point>> &contours);


void findContoursThresWhite(cv::Mat &src,
                            std::vector<std::vector<cv::Point>> &contours);

void getMaskThres(cv::Mat &src,
                  cv::Mat &dst);

void getMaskSobel(cv::Mat &src,
                  cv::Mat &dst);

void getMaskNormColor(cv::Mat &src,
                      cv::Mat &dst);

void getMaskInRangeColor(cv::Mat &src,
                         cv::Mat &dst);

void getMaskNormColorHardCode(cv::Mat &src,
                                cv::Mat &dst);

bool find_if_contours_cloes(std::vector<std::vector<cv::Point>> &contours);

#endif //X_CDS_GREENIMAGEPROCESS_H
