#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <math.h>
#include <fstream>

int restricting_radius = 4;
double TOUCH_THRESHHOLD = 0.6, THRESHHOLD = 0.4;

double g_right_distance, g_front_distance, g_left_distance;
geometry_msgs::Twist g_robot_velocity;
nav_msgs::OccupancyGrid msg;

bool start_pose_set = false;
geometry_msgs::Pose start_pose, current_pose;


int findAnyWall();
int turnLeft();
int findLostRightWall();
int followWall();

void goForward();
void goRight();
void goLeft();

void check_radius_restriction();

void rightIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    g_right_distance = msg->ranges[0];
    ROS_INFO("g_right_distance %f", g_right_distance);
}

void frontIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    g_front_distance = msg->ranges[0];
    ROS_INFO("g_front_distance %f", g_front_distance);
}

void leftIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    g_left_distance = msg->ranges[0];
    ROS_INFO("g_left_distance %f", g_left_distance);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &mesg)
{
    msg.info = mesg->info;
    msg.data = mesg->data;
}

void basePoseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if (!start_pose_set)
    {
        start_pose.position = msg->pose.pose.position;
        start_pose_set = true;
    }
    current_pose.position = msg->pose.pose.position;
}

void saveMap()
{
    double resolution = msg.info.resolution;
    int width = msg.info.width, height = msg.info.height;

    std::ofstream map_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map.txt"); //saving 2d map to file
    std::ofstream map_index_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map_index.txt");
    map_file << resolution << " " << width << " " << height << std::endl;

    int k = 0;
    for (int iRowIndex = 0; iRowIndex < height; ++iRowIndex)
    {
        for (int iColIndex = 0; iColIndex < width; ++iColIndex)
        {
            map_file << static_cast<int>(msg.data[k]) << "\t";
            map_index_file << static_cast<int>(msg.data[k]) << "(" << iRowIndex << "," << iColIndex << ")"
                           << "\t";
            k++;
        }
        map_file << std::endl;
        map_index_file << std::endl;
    }
    map_file.close();
    map_index_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_navigation");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Subscriber right_ir_sub = n.subscribe("base_scan_3", 10, rightIrCallback);
    ros::Subscriber front_ir_sub = n.subscribe("base_scan_1", 10, frontIrCallback);
    ros::Subscriber left_ir_sub = n.subscribe("base_scan_2", 10, leftIrCallback);

    ros::Subscriber map_sub = n.subscribe("map", 10, mapCallback);
    ros::Subscriber pose_sub = n.subscribe("base_pose_ground_truth", 10, basePoseCallback);

    int state = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        check_radius_restriction();

        switch (state)
        { // right wall following algorithm
        case 0:
        {
            ROS_INFO("findAnyWall");
            state = findAnyWall(); //finding any wall in front of the robot at the beginning of the maze
            break;
        }
        case 1:
        {
            ROS_INFO("turnLeft");
            state = turnLeft(); //turning left in case the robot is too close to the right wall and might soon crash into it
            break;
        }
        case 2:
        {
            ROS_INFO("findLostRightWall");
            state = findLostRightWall(); //turning right to find the wall(that made a right turn) that robot was just following
            break;
        }
        case 3:
        {
            ROS_INFO("followWall");
            state = followWall(); //following right wall, that goes straight
            break;
        }
        }
        velocity_pub.publish(g_robot_velocity);
        loop_rate.sleep();
    }

    saveMap();

    return 0;
}

void goForward()
{
    ROS_INFO("goForward");
    g_robot_velocity.linear.x = 1.0;
    g_robot_velocity.angular.z = 0;
}

void goRight()
{
    ROS_INFO("goRight");
    g_robot_velocity.linear.x = 0.25;
    g_robot_velocity.angular.z = -25.0;
}

void goLeft()
{
    ROS_INFO("goLeft");
    g_robot_velocity.linear.x = 0.1;
    g_robot_velocity.angular.z = 25.0;
}

int findAnyWall() //0
{
    if (g_right_distance > TOUCH_THRESHHOLD && g_front_distance > TOUCH_THRESHHOLD)
    {
        goForward();
        return 0;
    }
    if (g_right_distance < TOUCH_THRESHHOLD)
        return followWall();
    if (g_front_distance < TOUCH_THRESHHOLD)
        return turnLeft();
}

int turnLeft() //1
{
    if (g_right_distance < THRESHHOLD || g_front_distance < TOUCH_THRESHHOLD)
    {
        goLeft();
        return 1;
    }
    if (g_right_distance < TOUCH_THRESHHOLD)
        return followWall();
    return findLostRightWall();
}

int findLostRightWall() //2
{
    if (g_right_distance > TOUCH_THRESHHOLD && g_front_distance > TOUCH_THRESHHOLD)
    {
        goRight();
        return 2;
    }
    if (g_right_distance < TOUCH_THRESHHOLD)
        return followWall();
    if (g_front_distance < TOUCH_THRESHHOLD)
        return turnLeft();
}

int followWall() //3
{
    if (g_right_distance < TOUCH_THRESHHOLD && g_right_distance > THRESHHOLD && g_front_distance > TOUCH_THRESHHOLD)
    {
        goForward();
        return 3;
    }
    if (g_right_distance > TOUCH_THRESHHOLD)
        return findLostRightWall();
    if (g_right_distance < THRESHHOLD || g_front_distance < TOUCH_THRESHHOLD)
        return turnLeft();
}

void check_radius_restriction()
{
    double dist_x = pow(start_pose.position.x - current_pose.position.x, 2);
    double dist_y = pow(start_pose.position.y - current_pose.position.y, 2);
    double dist_from_start = sqrt(dist_x + dist_y);
    ROS_INFO("____distance from start %f", dist_from_start);
    if(dist_from_start > restricting_radius)
    {
        g_front_distance = 0;
    }
}