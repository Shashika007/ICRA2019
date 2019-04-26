#include "laser_pose.hpp"

nav_msgs::OccupancyGrid ORIG_MAP;
Mat MAP_IMG;
geometry_msgs::PoseStamped findEnemy(const sensor_msgs::PointCloud & pts, bool & okay);

geometry_msgs::PoseStamped findEnemy(const sensor_msgs::PointCloud & pts, bool & okay)
{
    cout << "find enemy\n";
    cv::Mat image(MAP_IMG.rows, MAP_IMG.cols, CV_8UC1, Scalar(0));
    vector<cv::Point> goodPts;
    goodPts.reserve(30);
    vector<cv::Point> goodPts2ndStage;
    goodPts2ndStage.reserve(20);
    cout << "1st stage filter\n";
    for(size_t i = 0; i < pts.points.size(); i++)
    {
        int xCor = pts.points[i].x * 10 / ORIG_MAP.info.resolution;
        int yCor = pts.points[i].y * 10 / ORIG_MAP.info.resolution;
        if (yCor >= MAP_IMG.rows || xCor >=MAP_IMG.cols)
        {
            ROS_WARN("%s", "point cloud out of bound, possibily caused by AMCL delay");
        }
        else if (MAP_IMG.at<int8_t>(yCor, xCor) < 80)
        {
            goodPts.push_back(cv::Point(xCor, yCor));
        }
        
    }
    cout << "2nd stage filter\n";
    for(size_t i = 0; i < goodPts.size(); i++)
    {
        double nearestDist = 99999.99;
        for(size_t j = 0; j < goodPts.size(); j++)
        {
            if(i!=j)
            {
                double dist = cv::norm(goodPts[i]-goodPts[j]);
                //cout << dist << "\n";
                if(dist < nearestDist)
                {
                    nearestDist = dist;
                }
            }
        }
        if (nearestDist < 50)
        {
            goodPts2ndStage.push_back(goodPts[i]);
        }
    }
    
    for(size_t i = 0; i < goodPts2ndStage.size(); i++)
    {
        cv::circle(image, cv::Point(goodPts2ndStage[i].x, goodPts2ndStage[i].y), 1, Scalar(255), 34); 
    }
    // imshow("Pts", image);
    // waitKey(10);
    cout << "before findcenter\n";
    bool good;
    auto center = findCenter(image, good);
    if (good)
    {
        okay = true;
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = ORIG_MAP.header.frame_id;
        msg.pose.orientation = geometry_msgs::Quaternion();
        msg.pose.position.x = center.x * ORIG_MAP.info.resolution / 10.0;
        msg.pose.position.y = center.y * ORIG_MAP.info.resolution / 10.0;
        return msg;
    }
    return geometry_msgs::PoseStamped();
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    cout << "scan callback\n";
    static laser_geometry::LaserProjection projector_;
    static tf::TransformListener listener_;
    static ros::NodeHandle n;
    static ros::Publisher pose = n.advertise<geometry_msgs::PoseStamped>("enemy_pose", 100);
    if (!listener_.waitForTransform(
            scan_in->header.frame_id,
            ORIG_MAP.header.frame_id,
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size() * scan_in->time_increment),
            ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud(ORIG_MAP.header.frame_id, *scan_in,
                                              cloud, listener_);
    bool found = false;
    auto enemyPose = findEnemy(cloud, found);
    if(found)
    {
        ROS_INFO("%s", "Found object");
        pose.publish(enemyPose);
    }

    // Do something with cloud.
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    //cout << "map callback\n";
    static bool flag = false;
    if (!flag)
    {
        flag = true;
        ORIG_MAP = nav_msgs::OccupancyGrid(*map);
        Mat temp = Mat(ORIG_MAP.info.height, ORIG_MAP.info.width, CV_8UC1, ORIG_MAP.data.data());
        cv::resize(temp, MAP_IMG, cv::Size(temp.cols * 10.0, temp.rows * 10.0 ), cv::INTER_CUBIC);
        ROS_INFO("%s%dx%d\n", "Processed Initial Costmap, size: ", MAP_IMG.rows, MAP_IMG.cols);
        imshow("costmap", MAP_IMG);
        waitKey(1000);
        static ros::NodeHandle n;
        static ros::Subscriber scansub = n.subscribe("scan", 30, scanCallback);
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_diff_checker");
    ROS_INFO("%s", "Initialized\n");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/global_costmap/global_costmap/costmap", 20, mapCallback);
    ros::spin();
    // ros::Rate loop_rate(20);

    // while (ros::ok())
    // {

    //     bool okay;
    //     auto goalMsg = findEnemy(msg, okay);
    //     if (okay)
    //     {
    //         pose.publish(goalMsg);
    //     }
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    return 0;
}