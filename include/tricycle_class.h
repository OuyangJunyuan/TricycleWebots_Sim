//
// Created by ou on 2020/11/5.
//

#ifndef MYWEBOTSDEMO_TRICYCLE_CLASS_H
#define MYWEBOTSDEMO_TRICYCLE_CLASS_H
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
using namespace webots;
using namespace std;


class TricycleWebotsCtrl{
private:
    /*
     * for webots
     */
    Robot *robot;
    Motor *motor[3];
    int timeStep;

    /*
     * for ros
     */
    ros::NodeHandle n;
    ros::Subscriber velosub,posisub;

    ros::ServiceClient set_lidar_client;
    webots_ros::set_int lidar_srv;
    ros::Subscriber sub_lidar_scan;

    void velocity_cmd_cb(const geometry_msgs::Twist::ConstPtr& ptr);
    void position_cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& ptr);
public:
    TricycleWebotsCtrl(string _VeloCmdTopicName,string _PosiCmdTopicName);
    int SimOnce(void);
    ~TricycleWebotsCtrl();
};

#endif //MYWEBOTSDEMO_TRICYCLE_CLASS_H
