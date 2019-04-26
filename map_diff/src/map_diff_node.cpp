#include "map_diff.hpp"

nav_msgs::OccupancyGrid ORIG_MAP;
Mat MAP_IMG;
bool flag = false;

geometry_msgs::PoseStamped findEnemy(nav_msgs::OccupancyGrid &result, bool &good);

geometry_msgs::PoseStamped findEnemy(nav_msgs::OccupancyGrid &result, bool &good)
{
    // result.info = currMap->info;
    // result.header.frame_id = currMap->header.frame_id;

    // for (size_t i = 0; i < ORIG_MAP.data.size(); i++)
    // {
    //     if (i >= currMap->data.size())
    //     {
    //         ROS_ERROR("%s", "Orig Map Larger Than Curr Map!\n");
    //         break;
    //     }
    //     auto temp = currMap->data[i] - ORIG_MAP.data[i];
    //     if (temp < 10)
    //     {
    //         temp = 0;
    //     }
    //     result.data.push_back(temp);
    // }
    //cv::Mat image(result.info.height, result.info.width, CV_8UC1, result.data.data());

    

    auto center = findCenter(image, good);
    if (good)
    {
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = currMap->header.frame_id;
        msg.pose.orientation = geometry_msgs::Quaternion();
        msg.pose.position.x = center.x * result.info.resolution;
        msg.pose.position.y = center.y * result.info.resolution;
        return msg;
    }
    return geometry_msgs::PoseStamped();
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
    static laser_geometry::LaserProjection projector_;
    static tf::TransformListener listener_;
    if (!listener_.waitForTransform(
            scan_in->header.frame_id,
            "/base_link",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size() * scan_in->time_increment),
            ros::Duration(1.0)))
    {
        return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_link", *scan_in,
                                              cloud, listener_);

    // Do something with cloud.
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map)
{
    //ROS_INFO("%s", "In callback\n");
    if (!flag)
    {
        ROS_INFO("%s", "Recieved first map\n");
        flag = true;
        ORIG_MAP = nav_msgs::OccupancyGrid(*map);
        MAP_IMG = Mat(ORIG_MAP.info.height, ORIG_MAP.info.width, CV_8UC1, ORIG_MAP.data.data());
        cv::resize(MAP_IMG, MAP_IMG, cv::Size(MAP_IMG.row * 10, MAP_IMG.col * 10), , , cv::InterpolationFlags::INTER_CUBIC);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_diff_checker");
    ROS_INFO("%s", "Initialized\n");
    ros::NodeHandle n;
    //ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("diff_map", 20);
    ros::Subscriber sub = n.subscribe("/global_costmap/global_costmap/costmap", 20, mapCallback);
    ros::Subscriber scansub = n.subscribe("scan", 30, scanCallback);

    ros::Publisher pose = n.advertise<geometry_msgs::PoseStamped>("enemy_pose", 100);
    ros::spinOnce();
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (flag)
        {
            //ROS_INFO("%s", "Start to broadcast map difference\n");
            nav_msgs::OccupancyGrid msg;
            bool okay;
            auto goalMsg = findEnemy(msg, okay);
            if (okay)
            {
                pose.publish(goalMsg);
            }
            //pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}