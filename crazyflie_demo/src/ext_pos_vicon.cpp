#include <cstdio>
#include <iostream>
#include <fstream>
#include <cmath>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "crazyflie_driver/GenericLogData.h"


ros::Publisher viconpos_pub;

geometry_msgs::PointStamped viconpos_msg;

std::string dir_path;
char time_buffer[80];

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

void saveLog(const crazyflie_driver::GenericLogData::ConstPtr& log)
{
    std::ofstream positionFile;
    positionFile.open (dir_path+"/logData_"+time_buffer+".txt", std::ios::app);
    positionFile << log->values[0] << ", " << log->values[1] << ", " << log->values[2] << "\r\n";
    positionFile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ext_pos_vicon");
    ros::NodeHandle n;
    
    dir_path = ros::package::getPath("crazyflie_hotdec");
    time_t rawtime;
    time(&rawtime);
    struct tm * timeinfo;
    timeinfo = localtime(&rawtime);
    strftime(time_buffer, 80, "%G%m%dT%H%M%S", timeinfo);
    
    viconpos_msg.header.seq = 0;
    
    viconpos_pub = n.advertise<geometry_msgs::PointStamped>("external_position", 1);

    ros::Subscriber vicon_pos = n.subscribe("/vrpn/hotdec_cf1/pose", 1, sendPos);
    ros::Subscriber waypoint = n.subscribe("/crazyflie/log1", 1, saveLog);
    
    ros::spin();
    return 0;
}
