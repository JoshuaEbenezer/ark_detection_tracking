#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp> 
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv_modules.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <math.h>

#include"iostream"
#include<stdio.h>

using namespace std;
using namespace cv; 


vector<Vec3f> hough(Mat img, float current_altitude)
{
Mat im;
im=img;
vector<Vec3f> circles;

 ROS_INFO("\n");
      if(im.empty())
         {       
           cout << "can not open " << endl;
          return circles;
         }

    Mat cimg,cimg1;
    medianBlur(im, cimg1, 5);
    cvtColor(cimg1, cimg, CV_BGR2GRAY);     

    int min_radius=pow(current_altitude,-1);          //TODO: find relationship bw altitude and min_radius
    int max_radius=current_altitude;                      //TODO: find relationship bw altitude and max_radius

    HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, 10,40, 25, min_radius, max_radius);      // TODO: change the last two parameters
                                                                                                                                                               // (min_radius & max_radius) to detect larger/smaller circles
                        
      for( size_t i = 0; i < circles.size(); i++ )
      {
        Vec3f c = circles[i];
        
        ROS_INFO("\nCircle: %d Centre: %f, %f Radius: %f", i+1, c[0],c[1],c[2]);
              
        circle( im, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, CV_AA);
        circle( im, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, CV_AA);
      }
  

return circles;
}



Rect roi(Mat img, vector<Vec3f> circles)
{
 Rect roi;
 vector<Point2f> centre;
 Point2f centre_of_frame;
 double min_dist=1000;
 double threshold;

 //TODO: give threshold a value
 threshold=1000;
 
 vector < double > dist;
 
  centre_of_frame=Point((img.cols)/2.0,(img.rows)/2.0);  //finding centre of image
     ROS_INFO(" %f %f ",centre_of_frame.x,centre_of_frame.y);   
          
          for(size_t i=0;i<circles.size();i++)        //finding robot that is furthest from the centre of the frame. This condition (for which robot to track) can be altered.
            {
           
               Vec3f c = circles[i];
               centre.push_back(Point(c[0],c[1]));             
               dist.push_back(norm(centre[i]-centre_of_frame));
               if(min_dist>dist[i])
                min_dist=dist[i];
            }
    if(min_dist<threshold)
         {
         
          for(size_t i=0;i<circles.size();i++)
            {
              if(dist[i]==min_dist)
               {
    
                Vec3f c = circles[i];
                roi=Rect(c[0]-c[2],c[1]+c[2],c[2],c[2]);
                rectangle(img, Point(c[0]-c[2],c[1]-c[2]),Point(c[0]+c[2],c[1]+c[2]),Scalar(0,255,0),3,CV_AA);
                ROS_INFO("\nRectangle Top left x: %d Top left y: %d Width: %d Height: %d",roi.x,roi.y,roi.width,roi.height);
                break;           
               }
            } 
          }

return roi;
}
