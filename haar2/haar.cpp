#include "haar.h"
#include "objconf.h"

//#define W 640
//#define H 140
std::vector<cv::Vec3b> color_;
cv::Vec3b black_(0, 0, 0);
cv::Vec3b white_(0, 0, 255);

void Haar::init(std::string source){
	std::string str = "TURN RIGHT";
	int flag = 1 ;
    right.Load(source+"/left2.xml");
	left.Load(source +"/left_ver2.xml");
//    right.Load(source+"/l1.xml");
//	left.Load(source +"/l2.xml");
	stop1.Load(source+ "/stop_ver10.xml");
	stop2.Load(source+ "/stop_ver11.xml");
	stop1.Init(cv::Size(640, 150), 1.2, cv::Size(10,10));
	stop2.Init(cv::Size(640, 150), 1.2, cv::Size(10,10));
	left.Init(cv::Size(590, 180), 1.2, cv::Size(2,2));
	right.Init(cv::Size(590, 180), 1.2, cv::Size(2,2));
    color_.push_back(cv::Vec3b(255,255,255));
    color_.push_back(cv::Vec3b(120, 255, 255));
    color_.push_back(cv::Vec3b(60, 255, 255));
    color_.push_back(cv::Vec3b(0, 255, 255));

}

void Haar::get_closest(cv::Vec3b &pixcel)
{
    if (pixcel[1] < obj::MIN_S)
    {
       pixcel = white_;
	   return;
    }

    if (pixcel[2] < obj::MIN_V || pixcel[2]>obj::MAX_V)
    {
	   pixcel = white_;
	   return;
    }
     int min_distance = 255;
    cv::Vec3b closest;
    for (auto color : color_)
    {
	   int distance = cv::abs(color[0] - pixcel[0]);

	   if (distance < min_distance)
	   {
		  min_distance = distance;
         // closest = color;
		  if (color[0] ==120)//select only red
		  {
			 closest = color;
		  }
		  else
		  {
			 closest = white_;
		  }

	   }
    }
    pixcel = closest;
}


cv::Rect Haar::get_mask_color(cv::Mat &src, cv::Mat &dst){
cv::Rect object_rect;
    cv::cvtColor(src, dst, cv::COLOR_BGR2HSV);

    for (int y = 0; y < dst.rows; ++y) {
	   for (int x = 0; x < dst.cols; ++x) {
		  cv::Vec3b current = dst.at<cv::Vec3b>(y, x);
		  get_closest(current);
		  dst.at<cv::Vec3b>(y, x) = current;
	   }
    }
    if(0){
    cv::Mat rgb;
    cv::cvtColor(dst, rgb, cv::COLOR_HSV2RGB);
    cv::imshow("color_normed", rgb);
    cv::waitKey(1);
}

 cv::Mat gray;
    cv::Mat hsv_channels[3];
    cv::split(dst, hsv_channels);
    gray = hsv_channels[1];
    cv::Rect rec =cut_contour(gray);
    return rec;
//    std::vector<std::vector<cv::Point> > contours;
//    cv::findContours(gray, contours, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
//
//    dst = cv::Mat::zeros(gray.size(), CV_8UC1);
//    int maxsize = 0;
//    cv::Rect _rect;
//    int area;
//
//    std::vector<std::vector<cv::Point> > hull;
//    for (int i = 0; i < contours.size(); i++) {
//	   if (cv::contourArea(contours[i]) > 100) {
//		  std::vector<cv::Point> sub_hull;
//		  cv::convexHull(cv::Mat(contours[i]), sub_hull, false);
//		  hull.push_back(sub_hull);
//		  _rect = cv::boundingRect(sub_hull);
//		  area =  _rect.area();
//		  if (area > maxsize){
//		  maxsize = area;
//		  object_rect = _rect;
//		  }
//	   }
//    }
//
//
////        find_if_contours_cloes(hull);
//    if(1){
//        drawContours(dst, hull, -1, 255, 1);
//    cv::imshow("dest",dst);
//    }

    //return is_object;

}
cv::Mat Haar::crop_h(cv::Mat frame, int W, int H){
	  cv::Mat out;
	  cv::Rect roi;
    roi.x = (640 - W)/2;
    roi.y = 20;
    roi.width = W;
    roi.height = frame.size().height - (480- H);
    frame(roi).copyTo(out);
    return out;
}
int Haar::detect_haar(cv::Mat frame , int NGABA_H){
  Detection::View image = frame;
  Detection::View i;
	Detection::Objects obRight, obLeft;
	if(NGABA_H % 100 ==0){
		str = "TURN RIGHT";
		flag = 1;
	}
    right.Detect(image, obRight);
    left.Detect(image, obLeft);

		if(obRight.size() >0 && obLeft.size() >0 ){
		 dis_x = std::abs(obRight[0].rect.Center().x - obLeft[0].rect.Center().x);
		 dis_y = std::abs(obRight[0].rect.Center().y - obLeft[0].rect.Center().y);
		if(dis_x <=40 && dis_y <=40){
			Simd::DrawRectangle(image, obLeft[0].rect, Simd::Pixel::Bgr24(0, 0, 255), 7);
			str = "TURN LEFT";
			flag = 2;
    	NGABA_H= NGABA_H- NGABA_H;
		}
	}
    // if(LEFT >= 2){
    // 	str = "TURN LEFT";
    // 	LEFT = 0;
    // 	NGABA_H= NGABA_H- NGABA_H;
    // }
    //cv::putText(frame, "TURN LEFT", obLeft[0].rect.TopLeft(), 3, 1, cv::Scalar(0,0,255),4);
    cv::putText(frame, str, Point(20, 20), 3, 1, cv::Scalar(0,0,255),3);
    return flag;
}

//int Haar::detect_haar(cv::Mat frame){
//	  cv::Mat outRight;
//
//    Detection::View image = frame;
//    Detection::View i;
//    Detection::Objects obRight, obLeft;
//    right.Detect(image, obRight);
//    left.Detect(image, obLeft);
//
//    for (size_t i = 0; i < obRight.size(); ++i){
//				frame(obRight[i].rect).copyTo(outRight);
//				cv::cvtColor(outRight ,outRight, cv::COLOR_RGB2HSV);
//				//cv::imshow("show right", outRight);
//				int RIGHT = find_blue(outRight);
//				std::cout<< "right ---" << RIGHT << std::endl;
//				//if(RIGHT >= 40){
//                Simd::DrawRectangle(image, obRight[i].rect, Simd::Pixel::Bgr24(0, 255, 0), 7);
//                flag = 1;
//					//str = "TURN RIGHT";
//				//}
//    }
//    for (size_t i = 0; i < obLeft.size(); ++i){
//				cv::Mat outLeft;
//				frame(obLeft[i].rect).copyTo(outLeft);
//				cv::cvtColor(outLeft ,outLeft, cv::COLOR_RGB2HSV);
//				//cv::imshow("show left", outLeft);
//				int LEFT  = find_blue(outLeft);
//				std::cout<< "left ----" << LEFT <<  std::endl;
//				//if(LEFT >= 40){
//        	    Simd::DrawRectangle(image, obLeft[i].rect, Simd::Pixel::Bgr24(0, 255, 255), 7);
//                flag = 2;
//					//str = "TURN LEFT";
//			//	}
//    }
//
//    //cv::putText(frame, "TURN LEFT", obLeft[0].rect.TopLeft(), 3, 1, cv::Scalar(0,0,255),4);
//    //cv::putText(frame, str, Point(20, 20), 3, 1, cv::Scalar(0,0,255),3);
//    return flag;
//}

//int Haar::dis_stop(cv::Mat frame, int status_){
//    cv::Mat out;
//		int min = 0;
//    Detection::View image = frame;
//    Detection::Objects objects;
//    stop.Detect(image, objects);
//		if(status_ == 1) min = 60;  // right
//		if(status_ == 2) min = 50;  //left
//		//std::cout<< "flag: ----" << status_ <<  std::endl;
//
//    for (size_t i = 0; i < objects.size(); ++i){
//        frame(objects[i].rect).copyTo(out);
//				cv::cvtColor(out ,out, cv::COLOR_RGB2HSV);
//				int STOP  = find_red(out);
//                Simd::DrawRectangle(image, objects[i].rect, Simd::Pixel::Bgr24(0, 255, 255), 7);
//                std::cout<< "percent------------" <<  STOP  << std::endl;
//
//                //cv::rectangle(frame, rect, cv::Scalar(255, 255, 255));
//				if( STOP > 50){
//                int pixel = objects[i].rect.BottomRight().y - objects[i].rect.TopRight().y;
//        		//cv::putText(frame, std::to_string((int) pixel ) + " cm", objects[i].rect.TopLeft(), 3, 1, cv::Scalar(0,0,255),4);
//						if(pixel >= 40) return 1;
//						return 2;
//				}
//				}
//				//cv::imshow("show", out);
//    return 0;
//}

////////////////////////////////////////// color /////////////////////////////////////
int Haar::dis_stop(cv::Mat frame){
    cv::Mat out, dst_;
    Detection::View image = frame;
    Detection::Objects objects1, objects2;
    stop1.Detect(image, objects1);
    stop2.Detect(image, objects2);

		if(objects2.size() >0&& objects1.size() >0 ){
             //frame(objects2[0].rect).copyTo(out);
             //cv::Rect rec = get_mask_color(out, dst_);
             //cv::rectangle(out, rec, cv::Scalar(255, 255, 255));
            // cv::imshow("color_normed", out);
             //cv::waitKey(1);
            //std::cout<< "rec height----" << rec.height<< std::endl;
            dis_x = std::abs(objects2[0].rect.Center().x - objects1[0].rect.Center().x);
            dis_y = std::abs(objects1[0].rect.Center().y - objects2[0].rect.Center().y);
            if(dis_x <=40 && dis_y <=40){
			Simd::DrawRectangle(image, objects2[0].rect, Simd::Pixel::Bgr24(0, 0, 255), 7);
                int pixel = objects1[0].rect.BottomRight().y - objects1[0].rect.TopRight().y;
                //if( pixel >=60) return 1;
                std::cout << " height stop-----" << pixel<< std::endl;
                if( pixel >=30) return 2;
                //return 2;
			}
}
    return 0;
}

////////////////////////////////////////////////////////////////


/////////////////////////haar /////////////////////////////////
//int Haar::dis_stop(cv::Mat frame){
//    cv::Mat out, dst_;
//    Detection::View image = frame;
//    Detection::Objects objects1, objects2;
//    stop1.Detect(image, objects1);
//    stop2.Detect(image, objects2);
//
//		if(objects2.size() >0 && objects1.size() >0){
//             //frame(objects2[0].rect).copyTo(out);
// //cv::imshow("color_normed", out);
//             //cv::waitKey(1);
//            dis_x = std::abs(objects2[0].rect.Center().x - objects1[0].rect.Center().x);
//            dis_y = std::abs(objects1[0].rect.Center().y - objects2[0].rect.Center().y);
//            if(dis_x <=40 && dis_y <=40){
//			Simd::DrawRectangle(image, objects2[0].rect, Simd::Pixel::Bgr24(0, 0, 255), 7);
//                int pixel = objects1[0].rect.BottomRight().y - objects1[0].rect.TopRight().y;
//                if( pixel >=50) return 1;
//                return 2;
//			}
//}
//    return 0;
//}

///////////////////////////////////////////////////////////////////////////////////

//int Haar::getArea(cv::Rect rect){
//	cv::Point top,bottom;
//	top = rect.tl();
//	bottom = rect.br();
//	int cols = bottom.x - top.x;
//	int rows = bottom.y - top.x;
//  return  cols * rows;
//}

//int Haar::find_red(cv::Mat &src){
//    unsigned char rangeH, rangeS, rangeV;
//		int STOP = 0;
//     for (int y = 0; y < src.rows; ++y) {
//        for (int x = 0; x < src.cols; ++x) {
//            if (src.at<cv::Vec3b>(y, x)[0] <= 160 && src.at<cv::Vec3b>(y, x)[0] > 10) {
//								STOP++;
//                rangeH = 0;
//                src.at<cv::Vec3b>(y, x)[0] = rangeH;
//                rangeS = 0;
//                rangeV = 0;
//                src.at<cv::Vec3b>(y, x)[1] = rangeS;
//                src.at<cv::Vec3b>(y, x)[2] = rangeV;
//							}
//        }
//    }
//    cv::Mat gray;
//    cv::Mat hsv_channels[3];
//    cv::split( src, hsv_channels);
//    gray = hsv_channels[1];
//		cv::Rect stop_ = cut_contour(gray );
//		//cv::imshow("RED", gray);
//		//return int( 100* getArea(stop_) /(src.rows* src.cols));
//		return  int(100* getArea(stop_) / (src.rows* src.cols));
//;
//}
//110 130
//95 135

//int Haar::find_blue(cv::Mat &src){
//    unsigned char rangeH, rangeS, rangeV;
//		 int COUNT = 0;
//     for (int y = 0; y < src.rows; ++y) {
//        for (int x = 0; x < src.cols; ++x) {
//            if (src.at<cv::Vec3b>(y, x)[0] <= 110 || src.at<cv::Vec3b>(y, x)[0] >= 130) {
//                COUNT++;
//                rangeH = 0;
//                src.at<cv::Vec3b>(y, x)[0] = rangeH;
//                rangeS = 0;
//                rangeV = 0;
//                src.at<cv::Vec3b>(y, x)[1] = rangeS;
//                src.at<cv::Vec3b>(y, x)[2] = rangeV;
//							}
//        }
//    }
//    cv::Mat gray;
//    cv::Mat hsv_channels[3];
//    cv::split( src, hsv_channels);
//    gray = hsv_channels[1];
//		cv::Rect signs_ = cut_contour(gray);
//		// cv::imshow("BLUE", gray);
//		return  int(100* getArea(signs_) / (src.rows* src.cols));
//}
cv::Rect Haar::cut_contour(cv::Mat &mask){
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours( mask, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
  if (contours.size() > 0){
  std::vector<std::vector<cv::Point>> contours_poly( contours.size() );
  std::vector<cv::Rect> boundRect( contours.size() );
  std::vector<cv::Point2f>center( contours.size() );
  std::vector<float>radius( contours.size() );
  for( size_t i = 0; i < contours.size(); i++ )
  {
      cv::approxPolyDP( cv::Mat(contours[i]), contours_poly[i], 1, true );
      boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
      cv::minEnclosingCircle( contours_poly[i], center[i], radius[i] );
  }
 int largest_area=0;
 int largest_contour=0;
 cv::Rect Rectt;
     for( int i = 0; i< contours.size(); i++ )
      {
       double a = cv::contourArea( contours[i],false);
       if(a>largest_area){
            largest_area=a;
            largest_contour=i;
            Rectt= cv::boundingRect(contours[i]);
            }
      }
		cv::rectangle(mask, boundRect[largest_contour], cv::Scalar(255, 255, 255));
	 return boundRect[largest_contour];
 }
 return cv::Rect(0,0,0,0);
}

float  Haar::getDir(float y, float x){
  float PI = 3.14;
  float angle = (float)std::atan2(y, x);
//  float deg = -0.019; //-(1+7/60)*PI/180  -1*7'
    float dec = (1+ 43 /60)*PI/180;
  angle += dec;
  if(angle < 0) angle += 2* PI;
  if(angle > 2 * PI) angle -= 2* PI;

  float deg = angle * 180 / PI;
  return deg;
// if((deg < 22.5)  || (deg > 337.5 ))  return "North";
// if((deg > 22.5)  && (deg < 67.5 ))   return "North-East";
// if((deg > 67.5)  && (deg < 112.5 ))  return "East";
// if((deg > 112.5) && (deg < 157.5 ))  return "South-East";
// if((deg > 157.5) && (deg < 202.5 ))  return "South";
// if((deg > 202.5) && (deg < 247.5 ))  return "SOuth-West";
// if((deg > 247.5) && (deg < 292.5 ))  return "West";
// if((deg > 292.5) && (deg < 337.5 ))  return "North-West";

}

namespace signhaar {
    Haar haar;
}
