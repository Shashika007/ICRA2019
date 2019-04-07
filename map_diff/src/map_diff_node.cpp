#include "map_diff.hpp"

nav_msgs::OccupancyGrid ORIG_MAP;
nav_msgs::OccupancyGridConstPtr currMap;
bool flag = false;

geometry_msgs::PoseStamped calculateDiff(nav_msgs::OccupancyGrid &result, bool &good);

geometry_msgs::PoseStamped calculateDiff(nav_msgs::OccupancyGrid &result, bool &good)
{
    result.info = currMap->info;
    result.header.frame_id = currMap->header.frame_id;

    for (size_t i = 0; i < ORIG_MAP.data.size(); i++)
    {
        if (i >= currMap->data.size())
        {
            ROS_ERROR("%s", "Orig Map Larger Than Curr Map!\n");
            break;
        }
        auto temp = currMap->data[i] - ORIG_MAP.data[i];
        if (temp < 10)
        {
            temp = 0;
        }
        result.data.push_back(temp);
    }

    // GridMap mapIn({"layer"});

    // mapIn.setGeometry(grid_map::Length(2.0, 1.0), 0.01);
    // mapIn["layer"];
    // const float minValue = -1.0;
    // const float maxValue = 1.0;
    cv::Mat image(result.info.height, result.info.width, CV_8UC1, result.data.data());
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
    // cv::imshow("Converted", image);
    // cv::waitKey(0);
    //GridMapCvConverter::toImage<unsigned char, 3>(mapIn, "layer", CV_8UC3, minValue, maxValue, image);
}

void mapDiffCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    //ROS_INFO("%s", "In callback\n");
    if (!flag)
    {
        ROS_INFO("%s", "Recieved first map\n");
        flag = true;

        ORIG_MAP = nav_msgs::OccupancyGrid(*msg);
    }
    currMap = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_diff_checker");
    ROS_INFO("%s", "Initialized\n");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<nav_msgs::OccupancyGrid>("diff_map", 20);
    ros::Subscriber sub = n.subscribe("/global_costmap/global_costmap/costmap", 20, mapDiffCallback);
    ros::Publisher goal = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 100);
    ros::spinOnce();
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (flag)
        {
            //ROS_INFO("%s", "Start to broadcast map difference\n");
            nav_msgs::OccupancyGrid msg;
            bool okay;
            auto goalMsg = calculateDiff(msg, okay);
            if (okay)
            {
                goal.publish(goalMsg);
            }
            pub.publish(msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}