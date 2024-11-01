#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>

using namespace std;

void move_turtle(ros::Publisher &pub_data, double linear_x, double linear_y, double angular_z = 0)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z != 0 ? (angular_z * M_PI) / 180 : 0;

    ros::Rate loop_rate(1); // 1 Hz

    pub_data.publish(msg);
    loop_rate.sleep();

    // Stop
    msg.linear.x = 0;
    msg.linear.y = 0;
    pub_data.publish(msg);
}

void respawn(ros::NodeHandle &nh, const std::string &name, float x, float y, float theta)
{
    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("kill");
    ros::ServiceClient spawn_client = nh.serviceClient<turtlesim::Spawn>("spawn");

    turtlesim::Kill kill_srv;
    kill_srv.request.name = name;
    kill_client.call(kill_srv);

    turtlesim::Spawn spawn_srv;
    spawn_srv.request.name = name;
    spawn_srv.request.x = x;
    spawn_srv.request.y = y;
    spawn_srv.request.theta = theta;
    spawn_client.call(spawn_srv);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_name");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    ros::Duration(1).sleep();

    respawn(nh, "turtle1", 1.5, 5, 0);
    ros::Duration(1).sleep();

    // R
    move_turtle(pub, 0, 1.5);

    move_turtle(pub, 0.5, 0);
    move_turtle(pub, 1, 0, -180);
    move_turtle(pub, 0.4, 0);

    move_turtle(pub, -0.75, 0.85);


    respawn(nh, "turtle1", 2.75, 5, 0);
    ros::Duration(1).sleep();
    
    // A

    move_turtle(pub, 0.5, 1.5);
    move_turtle(pub, 0.5, -1.5);
    move_turtle(pub, -0.2, 0.6);
    move_turtle(pub, -0.5, 0);

    respawn(nh, "turtle1", 4.1, 5, 0);
    ros::Duration(1).sleep();

    // S

    move_turtle(pub, 0.5, 0);
    move_turtle(pub, 1.1, 0, 180);
    move_turtle(pub, 0.3, 0);
    move_turtle(pub, 1.1, 0, -180);
    move_turtle(pub, 0.5, 0);

    respawn(nh, "turtle1", 5.4, 5.4, 0);
    ros::Duration(1).sleep();

    // Y

    move_turtle(pub, 0, 0, -90);
    move_turtle(pub, 1.6, 0, 180);
    move_turtle(pub, 1, 0);
    move_turtle(pub, -1.6, 0, -180);

    respawn(nh, "turtle1", 6.8, 5, 0);
    ros::Duration(1).sleep();

    // A

    move_turtle(pub, 0.5, 1.5);
    move_turtle(pub, 0.5, -1.5);
    move_turtle(pub, -0.2, 0.6);
    move_turtle(pub, -0.5, 0);

    respawn(nh, "turtle1", 8.5, 5, 0);
    ros::Duration(1).sleep();

    // N

    move_turtle(pub, 0, 1.5);
    move_turtle(pub, 1, -1.5);
    move_turtle(pub, 0, 1.5);

    return 0;
}
