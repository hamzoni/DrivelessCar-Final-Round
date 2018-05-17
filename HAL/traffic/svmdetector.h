#ifndef SVMPROCESSOR_H
#define SVMPROCESSOR_H

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/objdetect.hpp>
#include <iostream>

using namespace cv;
using namespace ml;
using namespace std;

namespace SVMConfig {
    int isTrain = 0;
    Mat trackImg = Mat();

    #ifdef scene // dark
    Scalar lower = Scalar(0,0,0);
    Scalar upper = Scalar(255,255,67);
    Mat closing = Mat(10, 10, CV_8U, Scalar(1));
    Mat opening = Mat(0, 0, CV_8U, Scalar(1));
    #else // light
    Scalar lower = Scalar(0,63,99);
    Scalar upper = Scalar(33,255,255);
    Mat closing = Mat(0, 0, CV_8U, Scalar(1));
    Mat opening = Mat(0, 0, CV_8U, Scalar(1));
    #endif
}

DetectorSVM::DetectorSVM();
void DetectorSVM::setLabels(string *labels)
void load();
void filter(Mat &hsv,Mat &mask);

Rect DetectorSVM::pooling(Mat &mask, Mat &out, Mat &gray);

vector<Rect> DetectorSVM::poolingMult(Mat &mask, Mat &gray, vector<Mat> &outs);

void on_track(int, void *) ;
void DetectorSVM::slider(int &val, int max, string title, string wname);
int DetectorSVM::predict(Mat &test);
string DetectorSVM::label(int &id);
string DetectorSVM::label(int &id);
int DetectorSVM::train();
int detect(Mat &img, Ptr<SVM> svm);
void DetectorSVM::preprocess(Mat &img, Mat &img2, Mat &hsv, Mat &gray); 
int DetectorSVM::detect(Mat &img);
vector<int> DetectorSVM::detectMult(Mat &img) ;
Mat DetectorSVM::draw(Mat frame, vector<Rect> boxes, String label);
void DetectorSVM::lsdirs(string path, vector<string> &folders);
void DetectorSVM::lsfiles(string path, vector<string> &files);
#endif // SVMPROCESSOR_H
