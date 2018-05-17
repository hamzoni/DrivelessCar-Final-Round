#include "svmdetector.h"

//#define scene
#define debug

int val[100];

Mat trackImg;
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

DetectorSVM::DetectorSVM()
{
    this->model = "/home/ubuntu/Desktop/DriverlessCarChallenge/traffic/train.txt";
//    this->model = "/home/taquy/Projects/python/ctr-crop/TestSVM/data-new/train.txt";
    this->data = "/home/taquy/Projects/python/ctr-crop/TestSVM/data-new/";
    this->sample = "/home/taquy/Projects/python/ctr-crop/TestSVM/data-new/1/81.jpg";

    this->load();


    string signs[] = {
        "Left",
        "Right",
        "Stop",
        "No sign detected"
    };
    this->setLabels(signs);


    // create window for slider testing
    #ifdef debug
        std::fill(val, val + 6, 0);
        namedWindow("mask", CV_WINDOW_AUTOSIZE);
        val[0] = 9;  this->slider(val[0], 200, "Opening", "mask");
        val[1] = 0; this->slider(val[1], 200, "Closing", "mask");

        val[2] = SVMConfig::lower[0]; this->slider(val[2], 255, "Lower B", "mask");
        val[3] = SVMConfig::lower[1]; this->slider(val[3], 255, "Lower R", "mask");
        val[4] = SVMConfig::lower[2]; this->slider(val[4], 255, "Lower G", "mask");

        val[5] = SVMConfig::upper[0]; this->slider(val[5], 255, "Upper B", "mask");
        val[6] = SVMConfig::upper[1]; this->slider(val[6], 255, "Upper R", "mask");
        val[7] = SVMConfig::upper[2]; this->slider(val[7], 255, "Upper G", "mask");
    #endif
}

void DetectorSVM::setLabels(string *labels) {
    for (int i = 0; i < labels->size(); i++) {
        this->labels.push_back(labels[i]);
    }
}

void DetectorSVM::load() {
    if (SVMConfig::isTrain == 0) {
        try {
            svm = SVM::create();
            svm = SVM::load(this->model);
        } catch (const std::exception& e) {
            cout << "Load model failed." << endl;
        }
    }
}


void filter(Mat &hsv, Mat &mask){
    #ifndef debug
    inRange(hsv, SVMConfig::lower, SVMConfig::upper, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, SVMConfig::opening);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, SVMConfig::closing);
    #else
    inRange(hsv, Scalar(val[2], val[3], val[4]), Scalar(val[5], val[6], val[7]), mask);
    Mat k(val[0], val[0], CV_8U, Scalar(1));
    Mat m(val[1], val[1], CV_8U, Scalar(1));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, k);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, m);
    #endif
    medianBlur(mask, mask, 3);
}

Rect DetectorSVM::pooling(Mat &mask, Mat &out, Mat &gray){

    vector<vector<Point>> contours;
    vector<Rect> boundRect(contours.size());
    vector<Vec4i> hierarchy;

    findContours(mask, contours, hierarchy, RETR_TREE , CHAIN_APPROX_SIMPLE, Point(0, 0) );

    if (contours.size() <= 0) return Rect();

    vector<vector<Point>> contours_poly(contours.size());
    vector<Point2f>center(contours.size());
    vector<float> radius(contours.size());

    int largest_area = 0;
    int largest_contour = 0;

    Rect rect;

    for(unsigned int i = 0; i< contours.size(); i++) {
        double a = contourArea( contours[i],false);
        if( a > largest_area){
            largest_area = a;
            largest_contour = i;
            rect = boundingRect(contours[i]);
        }
    }

    if(rect.area() < mask.size().height * mask.size().width) {

        #ifdef debug
            cv::rectangle(mask, rect, Scalar(255, 255, 255), 1, 8, 0);
            cv::rectangle(gray, rect, Scalar(255, 255, 255), 1, 8, 0);
           // imshow("gray", gray);
        #endif

        gray(rect).copyTo(out);
        return rect;
    }

}

vector<Rect> DetectorSVM::poolingMult(Mat &mask, Mat &gray, vector<Mat> &outs){
    vector<Rect> rects;
    vector<vector<Point>> contours;
    vector<Rect> boundRect(contours.size());
    vector<Vec4i> hierarchy;

    findContours(mask, contours, hierarchy, RETR_TREE , CHAIN_APPROX_SIMPLE, Point(0, 0) );

    if (contours.size() <= 0) return rects;



    for(unsigned int i = 0; i< contours.size(); i++) {
        Rect rect = boundingRect(contours[i]);
        int minw = 30;
        int minh = 30;
        double rti = rect.width / rect.height;
        if(rect.area() < mask.size().height * mask.size().width
            && rect.area() > minw * minh
//            && rti >= 0.8 && rti <= 1.2
        ) {
            rects.push_back(rect);

            Mat out;
            gray(rect).copyTo(out);
            outs.push_back(out);
        }

    }

    #ifdef debug
        for (int i = 0; i < rects.size(); i++) {
                Rect rect = rects[i];
                cv::rectangle(mask, rect, Scalar(255, 255, 255), 1, 8, 0);
                cv::rectangle(gray, rect, Scalar(255, 255, 255), 1, 8, 0);
        }
       // imshow("gray", gray);
    #endif
    return rects;
}

void on_track(int, void *) {
    if (SVMConfig::trackImg.empty()) return;
    Mat mask;
    filter(SVMConfig::trackImg, mask);
    //cv::imshow("mask", mask);

}

void DetectorSVM::slider(int &val, int max, string title, string wname) {
    title = title + " " + std::to_string(max);
    createTrackbar(title, wname, &val, max, on_track);
}

int DetectorSVM::predict(Mat &test) {

      resize(test,test,Size(64,48));
      HOGDescriptor hog(Size(64,48),Size(8,8), Size(4,4), Size(4,4), 9);
      vector<float> size_;
      hog.compute(test,size_);
      int col= size_.size();
      Mat testMat(1, col, CV_32FC1);
      vector<float> des;
      hog.compute(test,des);
      for (int i = 0; i < col; i++)
      testMat.at<float>(0,i)= des[i];

      return this->svm->predict(testMat);
}

string DetectorSVM::label(int &id) {
    if (id < 0 || id >= this->labels.size())
        id = this->labels.size() - 1;
    return this->labels[id];
}

int DetectorSVM::train(){

    if (SVMConfig::isTrain) return -1;

    int m = 0;
    int numfiles;
    Mat img, img0;
    vector<string> files;
    vector<string> folders;

    string f = this->data;

    img0 = imread(this->sample, 1);

    if(img0.empty()){
        cout << "Failed open image 0" << endl;
        return -1;
    }

    Mat re;

    resize(img0, re, Size(64,48));

    vector<float> size_;

    HOGDescriptor hog( Size(64,48), Size(8,8), Size(4,4), Size(4,4), 9);

    hog.compute(re, size_);

    this->lsdirs(f, folders);

    for(string s: folders) cout << s << endl;

    int num_imgs = 200;
    numfiles = num_imgs  * folders.size();
    Mat labels( numfiles, 1, CV_32S);
    Mat trainMat( numfiles, size_.size(), CV_32FC1);

    for(unsigned int x = 0; x < folders.size(); x++ ){
    this->lsfiles(folders[x], files);
    cout << folders[x] << endl;

    for(unsigned int y = 0; y< num_imgs; y++){
        cout << files[y] << endl;

        img = imread(files[y], 0);
        resize(img, img, Size(64,48));

        vector<float> des;
        hog.compute(img,des);
        for (unsigned int i = 0; i < size_.size(); i++)
            trainMat.at<float>(m,i)= des[i];
            labels.at<int>(m, 0) = x;
            m++;
        }
    }

    Ptr<SVM> svm = SVM::create();
    svm->setType(SVM::C_SVC);
    svm->setKernel(SVM::LINEAR);
    svm->setTermCriteria(TermCriteria(TermCriteria::MAX_ITER, 10000, 1e-6));
    svm->train(trainMat, ROW_SAMPLE, labels);
    svm->save(f + "train.txt");
    cout << "FINISHED" << endl;

}


void DetectorSVM::preprocess(Mat &img, Mat &img2, Mat &hsv, Mat &gray) {
    cv::flip(img, img, 1);
    cvtColor(img, img2, cv::COLOR_RGB2BGR);
    cvtColor(img2, gray, COLOR_BGR2GRAY);
    cvtColor(img2, hsv, COLOR_BGR2HSV);
}


int DetectorSVM::detect(Mat &img) {

    Mat mask, out, img2, hsv, gray;

    // must be BGR
    this->preprocess(img, img2, hsv, gray);
    filter(hsv, mask);

    #ifdef debug
    hsv.copyTo(SVMConfig::trackImg);
   // imshow("mask", mask);
    #endif

    // detect object
    Rect box = this->pooling(mask, out, gray);

    if(!out.empty()){

        // classification
        int id = predict(out);

        #ifdef debug

            // find circle
            vector<Vec3f> circles;

            cv::resize(out, out, Size(60, 60));

            vector<Rect> boxes;
            boxes.push_back(box);

            string lbl = this->label(id);
            cout << this->label(id) << endl;

            Mat rsl;
            cv::resize(out, rsl, Size(out.cols * 2, out.rows * 2));


            if (id != this->labels.size() - 1) {
                img = this->draw(img, boxes, lbl);

              //  imshow("region", rsl);
            } else {
                //imshow("region", rsl);
            }

        #endif

        return id;
    }

    return -1;
}

vector<int> DetectorSVM::detectMult(Mat &img) {
    Mat mask, out, img2, hsv, gray;

    this->preprocess(img, img2, hsv, gray);
    filter(hsv, mask);

    #ifdef debug
    hsv.copyTo(SVMConfig::trackImg);
   // imshow("mask", mask);
    #endif

    vector<Mat> outs;
    vector<Rect> objs = this->poolingMult(mask, gray, outs);
    vector<int> ids;

    if(!objs.size() == 0) {
        for (int i = 0; i < outs.size(); i++) {
            Mat out = outs[i];

            // classification
            int id = predict(out);

            #ifdef debug

                vector<Rect> boxes;
                boxes.push_back(objs[i]);


                string lbl = this->label(id);
                cout << this->label(id) << endl;


                if (id != this->labels.size() - 1) {
                    img = this->draw(img, boxes, lbl);
                }
            #endif
        }
    }

    return ids;

}

Mat DetectorSVM::draw(Mat frame, vector<Rect> boxes, String label) {

    // draw rects
    for( size_t i = 0; i < boxes.size(); i++ )
    {
        int x = boxes[i].x;
        int y = boxes[i].y;
        Point a(x, y);
        Point b(x + boxes[i].width, y + boxes[i].height);

        rectangle(frame, a, b, Scalar(0, 255, 0), 3);

        putText(frame, label, a, FONT_HERSHEY_SIMPLEX, 1, (0, 125, 255), 3, 0, false);
    }
    return frame;
}

void DetectorSVM::lsdirs(string path, vector<string> &folders){
    folders.clear();
    struct dirent *entry;

    DIR *dir = opendir(path.c_str());

    while((entry = readdir(dir))!= NULL){
        if ((strcmp(entry->d_name, ".") != 0) && (strcmp(entry->d_name, "..") != 0)) {
            string s =  string(path)  + string(entry->d_name) ;
            folders.push_back(s);
        }
    }

    closedir(dir);
    sort(folders.begin(),folders.end());
}


void DetectorSVM::lsfiles(string path, vector<string> &files) {

    files.clear();

    struct dirent *entry;

    DIR *dir = opendir(path.c_str());

    while((entry = readdir(dir))!= NULL){
        if ((strcmp(entry->d_name, ".") != 0) && (strcmp(entry->d_name, "..") != 0)) {
            string s =  string(path) + "/" + string(entry->d_name) ;
            files.push_back(s);
        }
    }

    closedir(dir);
    sort(files.begin(),files.end());
}
