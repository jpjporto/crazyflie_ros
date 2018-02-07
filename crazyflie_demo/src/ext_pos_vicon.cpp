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

void sendPos(const geometry_msgs::PoseStamped::ConstPtr& pose)
{

    viconpos_msg.header.frame_id = pose->header.frame_id;
    viconpos_msg.header.stamp = pose->header.stamp;
    viconpos_msg.header.seq += 1;
    viconpos_msg.point = pose->pose.position;
    //ros::Time stamp = ros::Time::now();

    //auto euler = q.toRotationMatrix().eulerAnglers(0, 1, 2); //This will be useful if we want to save the angles.

    /*std::ofstream positionFile;
    positionFile.open ("cf_pos.txt", std::ios::app);
    positionFile << stamp.toNSec() / 1000 << ", ";
    positionFile << point.x << ", " << point.y << ", " << point.z << "\n";
    positionFile.close();*/

    viconpos_pub.publish(viconpos_msg);
}

void sendWP(const geometry_msgs::Point& point)
{
    //m_cf.sendSetpoint(point.y, -point.x, 0.0, point.z*1000);
    wptarget_pub.publish(point);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ext_pos_vicon");
    ros::NodeHandle n;
    
    viconpos_msg.header.seq = 0;
    
    viconpos_pub = n.advertise<geometry_msgs::PointStamped>("external_position", 1);
    wptarget_pub = n.advertise<geometry_msgs::Point>("cmd_pos", 1);

    ros::Subscriber vicon_pos = n.subscribe("/vrpn_client_node/hotdec_cf1/pose", 1, sendPos);
    ros::Subscriber waypoint = n.subscribe("/waypoints", 1, sendWP);
    
    ros::spin();
    return 0;
}
