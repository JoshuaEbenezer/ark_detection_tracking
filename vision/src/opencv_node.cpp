#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "CMT.h"
#include "hough.h"

#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/CommandBool.h"
#include "ark_msgs/PidErrors.h"
#include <std_msgs/Float64.h>


#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdio>

#ifdef __GNUC__
#include <getopt.h>
#else
#include "getopt/getopt.h"
#endif

using cmt::CMT;
using cv::imread;
using cv::namedWindow;
using cv::Scalar;
using cv::VideoCapture;
using cv::waitKey;
using std::cerr;
using std::istream;
using std::ifstream;
using std::stringstream;
using std::ofstream;
using std::cout;
using std::min_element;
using std::max_element;
using std::endl;

   //Create a CMT object
    CMT cmt1;
    
    //Initialization bounding box
    Rect rect;
    
 
  
    vector < Vec3f > circles;
   //Get initial image
   
   int frame=0;
   int flag=0;





int display(Mat im, CMT & cmt)  //function to show the tracking result
{
    //Visualize the output
    //It is ok to draw on im itself, as CMT only uses the grayscale image
    for(size_t i = 0; i < cmt.points_active.size(); i++)
    {
        circle(im, cmt.points_active[i], 2, Scalar(255,0,0));
    }

    Point2f vertices[4];
    cmt.bb_rot.points(vertices);
    for (int i = 0; i < 4; i++)
    {
        line(im, vertices[i], vertices[(i+1)%4], Scalar(255,0,0));
    }

    

    return waitKey(5);                                           //rqt rvis 
}



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

    ros::NodeHandle nh_;
    ros::Publisher pid_pub;
    ros::Subscriber alt_sub;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

  }

    //Normal mode

    //Create window
    

  ~ImageConverter()
  {
    
  }
    AltHold()
    {
        pid_pub = nh_.advertise<ark_msgs::PidErrors>("/ark/pid_errors", 1000);
        alt_sub = nh_.subscribe("/mavros/global_position/rel_alt", 1000, &AltHold::altCb, this);
        target_height = 0;      
    }
    ~AltHold()
    {
    } 

 //callback function to store altitude in variable 'current_altitude'

    void altCb(const std_msgs::Float64::ConstPtr& msg)            //ALTITUDE
    {
        float current_altitude = msg->data;
        float diff = current_altitude - target_height;

        ark_msgs::PidErrors pid_error;
        pid_error.dz = diff;
        pid_pub.publish(pid_error);

        ROS_INFO("%.2f", diff);
    }   

    void imageCb(const sensor_msgs::ImageConstPtr& msg)      //IMAGE
   {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

   Mat im0;
   
   if(flag==0)   //to detect
    { 
              
        cv_ptr->image.copyTo(im0); // take im0 as cv_ptr->image

        frame++;                                                       //next frame index

        circles=hough(im0, current_altitude);            //find hough circles that are within the radius range prescribed by current_altitude
        
          if (circles.size()<5) 		//if no of circles detected is lesser than a particular threshold (which is set as 5 here) call hough again with the next frame of the video
           {
             flag=0;         
           }  
         else
          {
            rect=roi(im0,circles);        //call function roi to get bounding rectangles of the Hough circles     
   
       
             if(rect.area()==0)     //if proper bounding rectangle is not found
               {
                 flag=0;         /*set flag to 0 so that next frame is used for initialization and repeat until a proper bounding rectangle is found from hough*/    
               }                                          

             else
              {

                      //Convert im0 to grayscale
                   Mat im0_gray;

                   if (im0.channels() > 1) 
                    {
                     cvtColor(im0, im0_gray, CV_BGR2GRAY);
                    } 
                   else 
                    { 
                      im0_gray = im0;
                    }

                     //Initialize CMT
                     cmt1.initialize(im0_gray, rect);     //tracker initialized with the rectangles found from hough circles
    
                     flag=1;
             }
        }
    }
  else      //start tracking since detection is achieved
   {
        frame++;
        Mat im;
                
        cv_ptr->image.copyTo(im); //use next image in stream

        Mat im_gray;
          if (im.channels() > 1) 
           {
               cvtColor(im, im_gray, CV_BGR2GRAY);      //convert to gray for processing
           }
          else 
           {
             im_gray = im;
           }

        //Let CMT process the frame
        cmt1.processFrame(im_gray);

            if(cmt1.points_active.size()==0)           //if there's no active point then tracking cannot occur
              {
                   ROS_INFO("\nActive points not found");  
                   flag=0;     /*set flag to 0 so that next frame is used for initialization and repeat until a proper bounding rectangle is found from hough AND atleast 1 active point is found*/    
              }                                                       
            else
             {                                       
                   char key = display(im, cmt1);
                   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
                   image_pub_.publish(msg);                             //display the image
             }

   }
   
   
  }
   
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
