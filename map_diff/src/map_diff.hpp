#ifndef MAP_DIFF_H
#define MAP_DIFF_H
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <math.h>

using cv::cvtColor;
using cv::imread;
using cv::imshow;
using cv::Mat;
using cv::Scalar;
using cv::waitKey;

using std::cout;
using std::string;
using std::vector;

bool threshold(vector<cv::Point> pts);
bool threshold(vector<cv::Point> pts)
{
	return pts.size() < 20 || pts.size() > 80;
}
cv::Point2f findCenter(Mat &im, bool & good);

cv::Point2f findCenter(Mat &im, bool & good)
{

	cv::threshold(im, im, 10, 255, cv::ThresholdTypes::THRESH_OTSU);

	// imshow("Bin", im);

	// waitKey(0);
	vector<vector<cv::Point>> contours;
	cv::findContours(im, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_NONE);
	contours.erase(std::remove_if(contours.begin(), contours.end(), threshold), contours.end());
	//cv::drawContours(im, contours, -1, cv::Scalar(127), 2);
	Mat colored;
	cv::cvtColor(im, colored, cv::COLOR_GRAY2BGR);
    
    if (contours.size() == 0)
        {ROS_INFO("%s", "No Goal!");
        good = false;}
    
	for (size_t i = 0; i < contours.size(); i++)
	{
        good = true;
		cv::RotatedRect box = cv::minAreaRect(contours[i]);

		cv::circle(colored, box.center, 2, Scalar(0, 255, 0), 2);
		// cv::namedWindow("Target", cv::WINDOW_NORMAL);
		// cv::resizeWindow("Target", 800, 500);
		// rotated rectangle
       cv::Point2f rect_points[4]; 

	   box.points( rect_points );
	   bool notfilter = true;
	   for( int j = 0; j < 4; j++ )
	   {
		double res = cv::norm(rect_points[j] - rect_points[(j+1)%4]);
		if(res < 10 || res > 45)
		{
			notfilter = false;
			break;
		}
	   }
	   if(notfilter)
	   {
		return box.center;
	   }
	//    if (notfilter)
	//    {
	// 	for( int j = 0; j < 4; j++ )
	//    	{
		
    // 		line( colored, rect_points[j], rect_points[(j+1)%4], Scalar(0,255,0), 1, 8 );
	//    	}
	//    }
	   
	    

		// imshow("Target", colored);
		// waitKey(1);
        
	}
    return cv::Point2f();
	// imshow("Circle", colored);
	// waitKey(0);ffff
}

// class SubscribeAndPublish
// {
// public:
//   SubscribeAndPublish()
//   {
//     //Topic you want to publish
//     pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

//     //Topic you want to subscribe
//     sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
//   }

//   void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
//   {
//     PUBLISHED_MESSAGE_TYPE output;
//     //.... do something with the input and generate the output...
//     pub_.publish(output);
//   }

// private:
//   ros::NodeHandle n_;
//   ros::Publisher pub_;
//   ros::Subscriber sub_;

// };//End of class SubscribeAndPublish

#endif