#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <fstream>

#define NORTH 1
#define SOUTH 2
#define EAST 3
#define WEST 4

void loadMap();
void fillInWavefront();
void generatePath();
bool followPath(int direction, double distance);
void writeToFile();
void computeNeighbors(int x, int y);
bool validCell(int x, int y);
void enQueue(int x, int y);
void deQueue();
bool moveFront(double distance);
bool moveBack1Square();
bool turn90Clockwise();
bool turn180Clockwise();
bool turn90AntiClockwise();

int **world_map;

double resolution;
int width, height;

double TOUCH_THRESHHOLD = 0.3, THRESHHOLD = 0.2;

int *directions;
int *queue;
int rear = 0, front = 0;

// int start_x = 100, start_y = 100, //res=0.1
//     goal_x = 114, goal_y = 52;
int start_x = 50, start_y = 50,   //res=0.2
    goal_x = 56, goal_y = 26;

int north_x, north_y,
    south_x, south_y,
    east_x, east_y,
    west_x, west_y;

int current_orientation = EAST;

geometry_msgs::Pose2D initial_pose, current_pose;
geometry_msgs::Twist velocity;
double g_right_distance, g_front_distance, g_left_distance;
bool initial_pose_set = false;

void rightIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    g_right_distance = msg->ranges[0];
}

void frontIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    g_front_distance = msg->ranges[0];
}

void leftIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    g_left_distance = msg->ranges[0];
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose.theta = yaw;

    if (!initial_pose_set)
    {
        initial_pose = current_pose;
        initial_pose_set = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_navigation");
    ros::NodeHandle n;
    ros::Rate loop_rate(20);

    ros::Subscriber right_ir_sub = n.subscribe("base_scan_3", 10, rightIrCallback);
    ros::Subscriber front_ir_sub = n.subscribe("base_scan_1", 10, frontIrCallback);
    ros::Subscriber left_ir_sub = n.subscribe("base_scan_2", 10, leftIrCallback);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

    ROS_INFO("Loading map from the file");
    loadMap();

    ROS_INFO("Filling in the wavefront");
    fillInWavefront();

    ROS_INFO("Writing wavefront to the file");
    writeToFile();

    ROS_INFO("Generating path");
    generatePath();

    ROS_INFO("Following path");

    int i = 0;
    double dist = resolution;
    while (ros::ok())
    {
        while (directions[i] == directions[i + 1])
        {
            dist += resolution;
            i++;
        }
        if (followPath(directions[i], dist))
        {

            ROS_INFO("dir %d, dist %f", directions[i], dist);
            dist = resolution;
            i++;
        }
        cmd_vel_pub.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void loadMap()
{
    std::ifstream map_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map.txt");
    map_file >> resolution;
    map_file >> width;
    map_file >> height;

    world_map = new int *[height];

    for (int row = 0; row < height; ++row)
        world_map[row] = new int[width];

    for (int row = 0; row < height; ++row)
    {
        for (int column = 0; column < width; ++column)
        {
            int num;
            map_file >> num;
            num = num == 100 ? 1 : num == 0 ? 0 : -1;
            if (num == 1)
            {
                if (row > 0)
                {
                    world_map[row - 1][column] = 1;
                }
                if (column > 0)
                {
                    world_map[row][column - 1] = 1;
                }
                if (row < height - 1)
                {
                    world_map[row + 1][column] = 1;
                }
                if (column < width - 1)
                {
                    world_map[row][column + 1] = 1;
                }
            }
            world_map[row][column] = world_map[row][column] != 1 ? num : 1;
        }
    }
    map_file.close();

}

void fillInWavefront()
{
    queue = new int[height * width];
    world_map[goal_y][goal_x] = 2;
    enQueue(goal_x, goal_y);

    while (rear != front)
    {
        int next_to_examine_x = queue[front];
        int next_to_examine_y = queue[front + 1];

        int new_empty_cell_value = world_map[next_to_examine_y][next_to_examine_x] + 1;
        computeNeighbors(next_to_examine_x, next_to_examine_y);

        if (validCell(north_x, north_y) && world_map[north_y][north_x] == 0)
        {
            world_map[north_y][north_x] = new_empty_cell_value;
            enQueue(north_x, north_y);
        }

        if (validCell(south_x, south_y) && world_map[south_y][south_x] == 0)
        {
            world_map[south_y][south_x] = new_empty_cell_value;
            enQueue(south_x, south_y);
        }

        if (validCell(east_x, east_y) && world_map[east_y][east_x] == 0)
        {
            world_map[east_y][east_x] = new_empty_cell_value;
            enQueue(east_x, east_y);
        }

        if (validCell(west_x, west_y) && world_map[west_y][west_x] == 0)
        {
            world_map[west_y][west_x] = new_empty_cell_value;
            enQueue(west_x, west_y);
        }

        deQueue();
    }
}

void computeNeighbors(int x, int y)
{
    north_x = x;
    north_y = y - 1;

    south_x = x;
    south_y = y + 1;

    east_x = x + 1;
    east_y = y;

    west_x = x - 1;
    west_y = y;
}

void generatePath()
{
    int current_x = start_x;
    int current_y = start_y;
    directions = new int[world_map[start_y][start_x]];

    int smallestCell = world_map[start_y][start_x];

    int i = 0;
    while (smallestCell != world_map[goal_y][goal_x])
    {
        computeNeighbors(current_x, current_y);

        if (validCell(north_x, north_y) && world_map[north_y][north_x] < smallestCell)
        {
            smallestCell = world_map[north_y][north_x];
            current_x = north_x;
            current_y = north_y;
            directions[i] = 1;
        }

        else if (validCell(south_x, south_y) && world_map[south_y][south_x] < smallestCell)
        {
            smallestCell = world_map[south_y][south_x];
            current_x = south_x;
            current_y = south_y;
            directions[i] = 2;
        }
        else if (validCell(east_x, east_y) && world_map[east_y][east_x] < smallestCell)
        {
            smallestCell = world_map[east_y][east_x];
            current_x = east_x;
            current_y = east_y;
            directions[i] = 3;
        }
        else if (validCell(west_x, west_y) && world_map[west_y][west_x] < smallestCell)
        {
            smallestCell = world_map[west_y][west_x];
            current_x = west_x;
            current_y = west_y;
            directions[i] = 4;
        }
        ROS_INFO("%d %d", smallestCell, directions[i]);
        i++;
    }
}

bool validCell(int x, int y)
{
    if (x == -1 || y == -1 || x >= width || y >= height)
        return false;

    if (world_map[y][x] == 1 || world_map[y][x] == -1)
        return false;

    return true;
}

double getDistance(geometry_msgs::Pose2D pose_1, geometry_msgs::Pose2D pose_2)
{
    double x_diff = pose_1.x - pose_2.x;
    double y_diff = pose_1.y - pose_2.y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double getAngle(geometry_msgs::Pose2D pose_1, geometry_msgs::Pose2D pose_2)
{
    return abs(pose_1.theta - pose_2.theta);
}

bool followPath(int direction, double distance)
{
    switch (direction)
    {
    case NORTH: //1
        if(current_orientation == NORTH){
            if (moveFront(distance))
                return true;
        }
        else if (current_orientation == SOUTH)
        {
            if (turn180Clockwise())
                current_orientation = NORTH;
        }
        else if (current_orientation == EAST)
        {
            if(turn90AntiClockwise())
                current_orientation = NORTH;
        }
        else if (current_orientation == WEST)
        {
            if(turn90Clockwise())
                current_orientation = NORTH;
        }
        break;
    case SOUTH: //2
        if (current_orientation == SOUTH)
        {
            if (moveFront(distance))
                return true;
        }
        if (current_orientation == NORTH)
        {
            if (turn180Clockwise())
                current_orientation = SOUTH;
        }
        else if (current_orientation == EAST)
        {
            if (turn90Clockwise())
                current_orientation = SOUTH;
        }
        else if (current_orientation == WEST)
        {
            if (turn90AntiClockwise())
                current_orientation = SOUTH;
        }
        break;
    case EAST: //3
        if (current_orientation == EAST)
        {
            if (moveFront(distance))
                return true;
        }
        if (current_orientation == NORTH)
        {
            if (turn90Clockwise())
                current_orientation = EAST;
        }
        else if (current_orientation == SOUTH)
        {
            if(turn90AntiClockwise())
                current_orientation = EAST;
        }
        else if (current_orientation == WEST)
        {
            if(turn180Clockwise())
                current_orientation = EAST;
        }
        break;
    case WEST: //4
        if (current_orientation == WEST)
        {
            if (moveFront(distance))
                return true;
        }
        if (current_orientation == NORTH)
        {
            if(turn90AntiClockwise())
                current_orientation = WEST;
        }
        else if (current_orientation == SOUTH)
        {
            if(turn90Clockwise())
                current_orientation = WEST;
        }
        else if (current_orientation == EAST)
        {
            if(turn180Clockwise())
                current_orientation = WEST;
        }
        break;
    }
    return false;
}

void writeToFile()
{
    std::ofstream map_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map_wave.txt");
    int k = 0;
    for (int row = 0; row < height; ++row)
    {
        for (int column = 0; column < width; ++column)
        {
            //map_file << world_map[row][column] << "(" << row << "," << column << ")" << "\t";
            map_file << world_map[row][column] << "\t";
            k++;
        }
        map_file << std::endl;
    }
    map_file.close();
}

void enQueue(int x, int y)
{
    queue[rear] = x;
    queue[rear + 1] = y;
    rear += 2;
}

void deQueue()
{
    queue[front] = 0;
    queue[front + 1] = 0;
    front += 2;
}

bool moveFront(double distance)
{
    double covered_distance = getDistance(initial_pose, current_pose);
    if (covered_distance >= distance)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.5;
    velocity.angular.z = 0.0;
    return false;
}

bool turn90Clockwise()
{
    double rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI / 2)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.0;
    velocity.angular.z = 0.2;
    return false;
}

bool turn180Clockwise()
{
    double rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.0;
    velocity.angular.z = 0.2;
    return false;
}

bool turn90AntiClockwise()
{
    double rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI / 2)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.0;
    velocity.angular.z = -0.2;
    return false;
}