#ifndef MAP_DIFF_H
#define MAP_DIFF_H
#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>

using cv::cvtColor;
using cv::imread;
using cv::imshow;
using cv::Mat;
using cv::Scalar;
using cv::waitKey;

using std::cout;
using std::string;
using std::vector;

cv::Point2f findCenter(Mat &im, bool &good);

cv::Point2f findCenter(Mat &im, bool &good)
{
    //cout << "find center\n";
	vector<vector<cv::Point> > contours;
	cv::findContours(im, contours, cv::RetrievalModes::RETR_LIST, cv::ContourApproximationModes::CHAIN_APPROX_NONE);
	Mat colored;
	cv::cvtColor(im, colored, cv::COLOR_GRAY2BGR);

	if (contours.size() == 0)
	{
		good = false;
	}

	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::RotatedRect box = cv::minAreaRect(contours[i]);
		cv::circle(colored, box.center, 50, Scalar(0, 255, 0), 3);
		imshow("Circle", colored);
		waitKey(10);
		cv::Point2f rect_points[4];

		box.points(rect_points);

		good = true;
		return box.center;
	}
	return cv::Point2f();
	// imshow("Circle", colored);
	// waitKey(0);
}

#endif