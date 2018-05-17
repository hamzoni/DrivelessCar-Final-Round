#include "detectorobj.h"
#include "iostream"
#include "objconf.h"

cv::Rect roi;
cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
std::vector<cv::Vec3b> colors;
cv::Vec3b black(0, 0, 0);
cv::Vec3b white(0, 0, 255);
int object_color = 0;
Detector_Obj::Detector_Obj()
{
    roi.x = obj::ROI_X;
    roi.y = VIDEO_FRAME_HEIGHT/ 4;//obj::ROI_DIV_Y;
    roi.width = VIDEO_FRAME_WIDTH - obj::ROI_WIDTH_DOWN;
    roi.height = VIDEO_FRAME_HEIGHT - roi.y;
    colors.push_back(cv::Vec3b(255,255,255));
    colors.push_back(cv::Vec3b(120, 255, 255));
    colors.push_back(cv::Vec3b(60, 255, 255));
    colors.push_back(cv::Vec3b(0, 255, 255));
}
void get_closest(cv::Vec3b &pixcel)
{
    if   (pixcel[1] < obj::MIN_S)
    {
        pixcel = white;
        return;
    }

    if (pixcel[2] < obj::MIN_V || pixcel[2]>obj::MAX_V)
    {
        pixcel = white;
        return;
    }
    int min_distance = 255;
    cv::Vec3b closest;
    for (auto color : colors)
    {
        int distance = cv::abs(color[0] - pixcel[0]);

        if (distance < min_distance)
        {
            min_distance = distance;
//             closest = color;
            if (color[0] ==120)//select only red
            {
                closest = color;
            }
            else
            {
                closest = white;
            }

        }
    }
    pixcel = closest;

}
bool is_object_(cv::Mat croped){
    cv::cvtColor(croped,croped,cv::COLOR_BGR2GRAY);
    cv::Canny(croped, croped, 100, 255);
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(croped, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
    if(contours.size() > 5){
        return false;

    }
    return true;

}
void check_object_color(cv::Mat color){
    int mincount = 10;
    int count = 0;
    int color_ = 0;
    for (int y = 0; y < color.rows; ++y) {
        for (int x = 0; x < color.cols; ++x) {
            cv::Vec3b current = color.at<cv::Vec3b>(y, x);

            if(color_ == current[0]){
                count++;
            }
            if(color_= 0){
                color_ = current[0];

            }
            if(count >= mincount ){
                object_color = color_;
                return;
            }

        }
    }
}
bool get_mask_normaled_color(cv::Mat &src, cv::Mat &dst,cv::Rect &object_rect) {

    bool is_object = false;

    cv::cvtColor(src, dst, cv::COLOR_BGR2HSV);
    //dst = dst(roi);

    for (int y = 0; y < dst.rows; ++y) {
        for (int x = 0; x < dst.cols; ++x) {
            cv::Vec3b current = dst.at<cv::Vec3b>(y, x);
            get_closest(current);
            dst.at<cv::Vec3b>(y, x) = current;
        }
    }

    //DEBUG
    if(obj::DEBUG){
        cv::Mat rgb;
        cv::cvtColor(dst, rgb, cv::COLOR_HSV2RGB);
        cv::imshow("color_normed", rgb);
    }
    cv::Mat gray;
    cv::Mat hsv_channels[3];
    cv::split(dst, hsv_channels);
    gray = hsv_channels[1];
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(gray, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);

    dst = cv::Mat::zeros(gray.size(), CV_8UC1);
    int maxsize = 0;
    cv::Rect _rect;
    int area;

    std::vector<std::vector<cv::Point> > hull;
    for (int i = 0; i < contours.size(); i++) {
        if (cv::contourArea(contours[i]) > 100) {
            std::vector<cv::Point> sub_hull;
            cv::convexHull(cv::Mat(contours[i]), sub_hull, false);
            hull.push_back(sub_hull);

        }
    }
    for(int i = 0;i < hull.size(); i++){
        _rect = cv::boundingRect(hull[i]);
        //cv::imshow("asta",src.clone()(_rect));
 //       cv::waitKey(0);
        cv::Mat cloned =src.clone()(_rect);
        check_object_color(cloned);
        if(is_object_(cloned) || object_color == 120){
        object_rect = _rect;
        }

        is_object = true;
    }
//        find_if_contours_cloes(hull);
    if(obj::DEBUG){
        cv::drawContours(dst, hull, -1, 255, 1);
        cv::imshow("dest",dst);
    }

//         object_rect.y += roi.y;
//         object_rect.x += roi.x;

    return is_object;
}

bool get_object_type_color(cv::Mat color, cv::Rect &object_rect)
{
std::cout<<"color"<<"\n";
    cv::Mat mycolor;
    cv::resize(color,mycolor,cv::Size(160,120));
    mycolor = mycolor(roi);
    cv::Mat normed;

    //cv::RotatedRect object;
    bool is_object = get_mask_normaled_color(mycolor, normed, object_rect);
//	std::cout<<"is object : "<<is_object <<"\n";
//    cv::cvtColor(normed, normed, cv::COLOR_HSV2RGB);
//	if (is_object)
//	{
//	    cv::Point2f rect_points[4];
    //object.points(object_rect);
//	    object_rect = rect_points;
    //    for (int j = 0; j < 4; j++)
    //   cv::line(color, object_rect[j], object_rect[(j + 1) % 4], cv::Scalar(255, 255, 255), 1, 8);

// Return 4 point trong object_rects
//std::cout <<" object : "<< object_rect << "\n";
    if(obj::DEBUG){
        cv::rectangle(mycolor,object_rect,cv::Scalar(255,0,255));
        cv::imshow("color", mycolor);
    }

//	}
    if(is_object){
    object_rect.x+= roi.x;
    object_rect.y += roi.y;
    }
    return is_object;
}

cv::Rect get_color_in_box(cv::Mat color, cv::Rect maxsize_rect, cv::Vec3b &min, cv::Vec3b &max)
{
    min = cv::Vec3b(255, 255, 255);
    max = cv::Vec3b(0, 0, 0);
    cv::Vec3b current;
    //cv::imshow("clor", color);
    int w = maxsize_rect.width * 0.1, h = maxsize_rect.height*0.1;
    int x = maxsize_rect.x + maxsize_rect.width / 2, y = maxsize_rect.y + maxsize_rect.height / 2;

    //std::cout <<"X Y :"<< maxsize_rect <<"\n";

    if (y + h > color.size().height)
    {
        h = color.size().height - y;
    }
    if (x + w > color.size().width)
    {
        w = color.size().width - x;
    }
    for (int i = y; i < y + h; ++i)
    {
        for (int j = x; j < x + w; ++j)
        {
            current = color.at<cv::Vec3b>(i, j);
            for (int t = 0; t < 3; ++t)
            {
                if (current[t] > max[t])
                {
                    max[t] = current[t];
                }
                if (current[t] < min[t])
                {
                    min[t] = current[t];
                }
            }
        }
    }
    return cv::Rect(x, y, w, h);
}

cv::Rect Detector_Obj::get_color_rect(cv::Mat color, cv::Rect maxsize_rect, int maxsize)
{
    cv::Rect max_color_rect;
    cv::Mat color_clone;
    if (maxsize > 0)
    {
        color_clone = color.clone();

        if (maxsize_rect.width + maxsize_rect.x > color.size().width)
        {
            maxsize_rect.width = color.size().width - maxsize_rect.x;
        }
        if (maxsize_rect.height + maxsize_rect.y > color.size().height)
        {
            maxsize_rect.height = color.size().height - maxsize_rect.y;
        }

        cv::Mat binary;
        cv::Vec3b min, max;
        cv::Rect small_rect = get_color_in_box(color_clone, maxsize_rect, min, max);
        cv::inRange(color_clone, min, max, binary);

//	     std::cout<<min << "\t"<<max <<"\n";
//		cv::imshow("clolrcln",binary);
        cv::blur(binary, binary, cv::Size(3, 3));

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
        int max_color_rect_size = 0, area;
        cv::Rect bounding_rect;
        std::vector<cv::Point> contours_poly;

        for (size_t i = 0; i < contours.size(); i++) {
            cv::approxPolyDP(contours[i], contours_poly, 3, false);
            bounding_rect = cv::boundingRect(contours_poly);
            //int distance = std::sqrt(pow(bounding_rect.x - maxsize_rect.x, 2) + pow(bounding_rect.y - maxsize_rect.y, 2));
            bool is_bounding = (bounding_rect.tl().x < small_rect.tl().x)
                               && (bounding_rect.tl().y < small_rect.tl().y)
                               && (bounding_rect.br().x > small_rect.br().x)
                               && (bounding_rect.br().y > small_rect.br().y);
            area = bounding_rect.area();
            if (area > max_color_rect_size &&area < 4000 && is_bounding)
            {
                max_color_rect_size = area;
                max_color_rect = bounding_rect;
            }
        }
    }
    return max_color_rect;
}
bool Detector_Obj::type_roi_floor(cv::Mat depthImg, cv::Mat color, cv::Rect& object_rect)
{
//std::cout<<"object roi : " <<obj::ROI_DIV_Y<<"\n";

    cv::resize(color, color, cv::Size(160, 120));
    cv::resize(depthImg, depthImg, cv::Size(160, 120));
    //cv::imshow("tasd",depthImg);
    cv::Mat src = depthImg(roi);
//    cv::blur(src, src, cv::Size(3, 3));
    int w = src.size().width, h = src.size().height;

    cv::Mat binary;
    cv::inRange(src, cv::Scalar(obj::ROI_FLOOR_MIN_DEPTH), cv::Scalar(obj::ROI_FLOOR_MAX_DEPTH), binary);
    //    cv::threshold(depthImg, binary, 70, 72, cv::THRESH_BINARY);
    cv::rectangle(binary, cv::Point(0, h/3.5 /*obj::ROI_FLOOR_DIV*/), cv::Point(w, h), cv::Scalar(255), 1);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary, contours, CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    cv::Rect bounding_rect;
//    cv::blur(binary, binary, cv::Size(3,3));
    if(obj::DENOISE){
        cv::erode(binary,binary,element);
        cv::dilate(binary,binary,element);
    }

    int maxsize = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> contours_poly;
        cv::approxPolyDP(contours[i], contours_poly, 3, false);
        bounding_rect = boundingRect(contours_poly);
        int area = bounding_rect.area();
        if (area > obj::MIN_AREA)
        {
            //std::cout<<bounding_rect.width / bounding_rect.height<<"\n";
            //if (bounding_rect.width / bounding_rect.height < 5) {
            // std::cout<<bounding_rect.width / bounding_rect.height<<"\n";
            bounding_rect.y += roi.y;
            bounding_rect.x += roi.x;
            bounding_rect.x += 5;
            if (area > maxsize)
            {
                object_rect = bounding_rect;
                maxsize = area;
            }
            //cv::rectangle(depthImg, bounding_rect, cv::Scalar(255));
            //}
        }
    }
    if(obj::CHECK_COLOR == 1){
        object_rect = get_color_rect(color, object_rect, maxsize);
    }
    object_rect.height= object_rect.width;

    if(obj::DEBUG ==1)
    {
        cv::imshow("Depth",depthImg);
        cv::imshow("binary", binary);
        if(maxsize> 0){
            rectangle(color, object_rect, cv::Scalar(255, 255, 0));
            imshow("depthcolor",color);
        }

    }



    if (maxsize>0)
    {
//        std::cout<<"return TRUE" << "\n";
        return true;
    }
    //  std::cout<<"return FALSE" << maxsize <<"\n";



    return false;
}

bool Detector_Obj::get_object_depth(cv::Mat depthImg, cv::Mat color, cv::Rect &object_rect)
{
//    if(obj::DEBUG){
//    cv::imshow("original_depth",depthImg);
//    }
    if(obj::TYPE==1)
    {
        return type_canny(depthImg,color,object_rect);
    }
    else if (obj::TYPE ==2)
    {
        return type_roi_floor(depthImg, color,object_rect);
    }
    else if(obj::TYPE == 3)
    {
        return get_object_type_color(color,object_rect);
    }
    else{
        return false;
    }

}

bool    Detector_Obj::type_canny(cv::Mat depthImg, cv::Mat color, cv::Rect &object_rect)
{
    cv::resize(color, color, cv::Size(160, 120));
    cv::resize(depthImg, depthImg, cv::Size(160, 120));
    //imshow("dep", depthImg);

    std::vector<cv::Rect> objects = get_objects(depthImg);

    // move box
    //cv::Rect maxsize_rect;
    int maxsize = 0, area;

    for (size_t i = 0; i < objects.size(); i++) {
        area = objects[i].area();
        if (area > maxsize)
        {
            maxsize = area;
            object_rect = objects[i];
        }
    }
    if(obj::CHECK_COLOR == 1){
        object_rect = get_color_rect(color, object_rect, maxsize);
    }
    object_rect.height = object_rect.width;
    if (maxsize > 0) {
        if(obj::DEBUG)
        {
            rectangle(color, object_rect, cv::Scalar(255, 255, 0));
            imshow("depthcolor",color);
        }
        return true;
    }



    return false;
}

bool is_no_depth(cv::Mat depth, cv::Rect rect)
{
    int value = depth.at<char>(rect.y + rect.height / 2, rect.x + rect.width / 2);
    return value == 39;

}

std::vector<cv::Rect> Detector_Obj::get_objects(cv::Mat depth)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat src = depth(roi);
    //tests

    cv::Canny(src, src, 100, 255);
    cv::findContours(src, contours, CV_RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Rect> bounding_rects;
    cv::Rect bounding_rect;

    for (size_t i = 0; i < contours.size(); i++) {
        std::vector<cv::Point> contours_poly;
        cv::approxPolyDP(contours[i], contours_poly, 3, false);
        bounding_rect = boundingRect(contours_poly);

        is_no_depth(depth, bounding_rect);
        if (bounding_rect.width*bounding_rect.height > obj::MIN_AREA)
        {
            if (bounding_rect.width / bounding_rect.height < 3 && !is_no_depth(depth, bounding_rect)) {
                bounding_rect.y += roi.y;
                bounding_rect.x += roi.x;
                bounding_rects.push_back(bounding_rect);
            }
        }
    }

//
//
//    std::vector<std::vector<cv::Point> > hull;
//    for (int i = 0; i < contours.size(); i++) {
//	   if (cv::contourArea(contours[i]) > 100) {
//		  std::vector<cv::Point> sub_hull;
//		  cv::convexHull(cv::Mat(contours[i]), sub_hull, false);
//		  hull.push_back(sub_hull);
//
//	   }
//    }
//    if(hull.size()>0){
//        bounding_rect = cv::boundingRect(hull[0]);
//
//
//
//    }




    if(obj::DEBUG)
    {
//    drawContours(src, hull, -1, 255, 1);
        cv::imshow("dest",src);
    }


    return bounding_rects;
}




namespace objdetect {
    Detector_Obj detector;
}
