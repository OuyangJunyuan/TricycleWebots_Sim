//
// Created by ou on 2020/11/5.
//
#include "tricycle_class.h"
#include <boost/bind.hpp>
#define WHEEL_JOINT0_NAME "base_wheel_0_joint"
#define WHEEL_JOINT1_NAME "base_wheel_1_joint"
#define WHEEL_JOINT2_NAME "base_wheel_2_joint"

#define FRONT_WHEEL_IDX (0)
#define LEFT_WHEEL_IDX  (1)
#define RIGHT_WHEEL_IDX (2)

#define CHASSIS_WHEEL_R (0.4*1.414/2+0.05)
#define WHEEL_R (0.127/2.0*2.5)

#define WHEEL_PID_P (10 )
#define WHEEL_PID_I (0)
#define WHEEL_PID_D (0)

float v0,v1,v2;
void base_math_model(float vx,float vy,float az,float *v0,float *v1,float *v2){
    *v0=( vx                            + CHASSIS_WHEEL_R*-az )/WHEEL_R;
    *v1=( vx*(-0.5) + vy*(1.732050/2.0) + CHASSIS_WHEEL_R*-az )/WHEEL_R;
    *v2=( -vx*(0.5)  - vy*(1.732050/2.0) + CHASSIS_WHEEL_R*-az )/WHEEL_R;
}

void TricycleWebotsCtrl::velocity_cmd_cb(const geometry_msgs::Twist::ConstPtr& ptr){
    base_math_model(ptr->linear.x,ptr->linear.y,ptr->angular.z,&v0,&v1,&v2);
    static int id=0;
    ROS_INFO("msg_id:%d  v0:%5f v1:%5f  v2:%5f",id++,v0,v1,v2);
    motor[FRONT_WHEEL_IDX]->setVelocity(v0);
    motor[LEFT_WHEEL_IDX]->setVelocity(v1);
    motor[RIGHT_WHEEL_IDX]->setVelocity(v2);
}
void TricycleWebotsCtrl::position_cmd_cb(const geometry_msgs::PoseStamped::ConstPtr& ptr){
//    base_math_model(ptr->linear.x,ptr->linear.y,ptr->angular.z,&v0,&v1,&v2);
//    static int id=0;
//    id++;
//    ROS_INFO("msg_id:%d  v0:%5f v1:%5f  v2:%5f",id,v0,v1,v2);
//    motor[FRONT_WHEEL_IDX]->setVelocity(v0);
//    motor[LEFT_WHEEL_IDX]->setVelocity(v1);
//    motor[RIGHT_WHEEL_IDX]->setVelocity(v2);
}

TricycleWebotsCtrl::TricycleWebotsCtrl(string _VeloCmdTopicName,string _PosiCmdTopicName){
    robot = new Robot();
    timeStep = (int)robot->getBasicTimeStep();

    motor[0] = robot->getMotor(WHEEL_JOINT0_NAME);
    motor[1] = robot->getMotor(WHEEL_JOINT1_NAME);
    motor[2] = robot->getMotor(WHEEL_JOINT2_NAME);
    motor[0]->setPosition(INFINITY);
    motor[1]->setPosition(INFINITY);
    motor[2]->setPosition(INFINITY);
    motor[0]->setControlPID(WHEEL_PID_P,WHEEL_PID_I,WHEEL_PID_D);
    motor[1]->setControlPID(WHEEL_PID_P,WHEEL_PID_I,WHEEL_PID_D);
    motor[2]->setControlPID(WHEEL_PID_P,WHEEL_PID_I,WHEEL_PID_D);
    motor[FRONT_WHEEL_IDX]->setVelocity(0);
    motor[LEFT_WHEEL_IDX]->setVelocity(0);
    motor[RIGHT_WHEEL_IDX]->setVelocity(0);
    

//    velosub = n.subscribe("/cmd_vel", 1000, &TricycleWebotsCtrl::velocity_cmd_cb,this); //两者都可
    velosub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1000, boost::bind(&TricycleWebotsCtrl::velocity_cmd_cb,this,_1));
    posisub = n.subscribe("/move_base_simple/goal", 1000, &TricycleWebotsCtrl::position_cmd_cb,this);

    set_lidar_client = n.serviceClient<webots_ros::set_int>("/TricycleSensor/Hokuyo_URG_04LX/enable");
    lidar_srv.request.value = timeStep;

    cout<<"Constructor:Start Enable Lidar"<<endl;
    while (!(set_lidar_client.call(lidar_srv) && lidar_srv.response.success));
    cout<<"Constructor:End Enable Lidar"<<endl;
}
TricycleWebotsCtrl::~TricycleWebotsCtrl() {
    delete robot;
}
int TricycleWebotsCtrl::SimOnce() {
    return robot->step(timeStep);
}