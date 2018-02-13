#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cstdbool>
#include <fstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Point.h"

#define LAND_RATE    1500
#define TAKEOFF_RATE 750
#define MSG_RATE     1       // in ms
#define MIN_HEIGHT   0.15    // in m
#define PRINT_RATE   10
#define TAKEOFF_H    0.75


void compute_trajectory();

geometry_msgs::Point cfSetpoint1, cfSetpoint2, cfSetpoint3;
geometry_msgs::Point tempSetpoint1;
geometry_msgs::Point landSetpoint, takeoffSetpoint;

std::thread cf_thread;

typedef enum setpoint_cmds
{
    LAND,
    TAKEOFF,
    CIRCLE,
    GOTO,   //Probably need a better name
    SQUARE
} SETPOINT_CMDS;

uint8_t msg_flag;
uint8_t isFlying;
SETPOINT_CMDS setpoint_type;
uint32_t landcount, takeoffcount, circcount;
bool newGOTO;

double A, f;
geometry_msgs::Point circ_offset, wpmsg;

uint8_t seq1, seq2, seq3, setpointSeq;

ros::Publisher wptarget_pub;

void run()
{

    while(ros::ok())
    {
        compute_trajectory();
        wpmsg = cfSetpoint1;
        wptarget_pub.publish(wpmsg);

        std::this_thread::sleep_for(std::chrono::milliseconds(MSG_RATE));
    }
    
    
    // Turn off engines when done with program
    ROS_INFO("Sending turn off commands");
    wpmsg.x = 0;
    wpmsg.y = 0;
    wpmsg.z = 0;
    for(int i=0; i<100; i++)
    {
        wptarget_pub.publish(wpmsg);
        std::this_thread::sleep_for(std::chrono::microseconds(500));
    }
}

void sendSetpoint1(const geometry_msgs::Point& point)
{
    std::cout << "Going to Point (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    if(!isFlying && point.z > 0.0)
    {
        cfSetpoint1.x = 0.0;
        cfSetpoint1.y = 0.0;
        tempSetpoint1.z = point.z;
        isFlying = 1;
        setpoint_type = TAKEOFF;
        //takeoffcount = TAKEOFF_H*TAKEOFF_RATE/point.z;
        takeoffcount = 0;
    }
    else if(isFlying && point.z <= 0.0)
    {
        setpoint_type = LAND;
        landcount = 0;
    }
    else
    {
        setpoint_type = GOTO;
        cfSetpoint1 = point;
        newGOTO = true;
    }
}

void circleSetpoint(const geometry_msgs::Point& point)
{
    std::cout << "Generating circle with A = " << point.x << ", freq = " << point.y << std::endl;
    A = point.x;
    f = point.y;
    circcount = 0;
    circ_offset = cfSetpoint1;
    setpoint_type = CIRCLE;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "setpoint_pub");
    ros::NodeHandle n;
    
    
    ros::Subscriber setpoint1 = n.subscribe("/setpoint1", 1, sendSetpoint1);
    ros::Subscriber circSetpoint = n.subscribe("/CircleSetpoint", 1, circleSetpoint);
    
    wptarget_pub = n.advertise<geometry_msgs::Point>("crazyflie/broadcast_setpoint", 1);
    
    msg_flag = 1;
    isFlying = 0;
    setpointSeq = 0;
    std::memset(&cfSetpoint1, 0, sizeof(cfSetpoint1));
    
    cf_thread = std::thread(run);
    
    ros::spin();
    cf_thread.join();
    
    ROS_INFO("Done");
}

void compute_trajectory()
{
    switch(setpoint_type)
    {
        case LAND:
            landcount++;
            cfSetpoint1.z = tempSetpoint1.z*(LAND_RATE-landcount)/LAND_RATE;
            if((landcount > LAND_RATE) || cfSetpoint1.z <= MIN_HEIGHT)
            {
                cfSetpoint1.z = 0;
                isFlying = 0;
                setpoint_type = GOTO;
            }
            break;
            
        case TAKEOFF:
            if(takeoffcount < TAKEOFF_H*TAKEOFF_RATE/tempSetpoint1.z)
            {
                // Send x and y setpoints first so the velocity setpoints will equal zero
                // once we send the first nonzero z setpoint.
                cfSetpoint1.z = 0;
                takeoffcount++;
            }
            else if(takeoffcount < TAKEOFF_RATE)
            {
                cfSetpoint1.z = fmax(tempSetpoint1.z*takeoffcount/TAKEOFF_RATE, TAKEOFF_H);
                takeoffcount++;
            }
            else
            {
                cfSetpoint1.z = tempSetpoint1.z;
                isFlying = 1;
                setpoint_type = GOTO;
            }
            break;
            
        case GOTO:
            
            break;
            
        case CIRCLE:
            // For now we assume that each step takes exactly 2ms
            cfSetpoint1.x  = A*sin(f * 0.002 * circcount) + circ_offset.x;
            cfSetpoint1.y  = A*(cos(f * 0.002 * circcount) - 1.0) + circ_offset.y;
            circcount++;
            break;
            
        default:
            break;
    }
}
