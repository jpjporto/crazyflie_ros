#include <cstdio>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "ros/package.h"


ros::Publisher viconpos_pub;
ros::Publisher wptarget_pub;

geometry_msgs::PointStamped viconpos_msg;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ext_pos_vicon");
    ros::NodeHandle n;
    
    ros::Rate loop_rate(100);
    
    viconpos_msg.header.seq = 0;
    viconpos_msg.header.frame_id = "vicon";
    //viconpos_msg.header.stamp = 0;
    viconpos_msg.point.x = 0.01;
    viconpos_msg.point.y = -0.31;
    viconpos_msg.point.z = 0.10;
    
    viconpos_pub = n.advertise<geometry_msgs::PointStamped>("crazyflie/external_position", 1);
    
    while(ros::ok())
    {
        viconpos_msg.header.seq += 1;
        viconpos_pub.publish(viconpos_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
