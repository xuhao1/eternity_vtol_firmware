//
// Created by xuhao on 2016/4/14.
//

#ifndef ETERNITY_FC_SIMULATOR_H
#define ETERNITY_FC_SIMULATOR_H

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <dji_sdk/Acceleration.h>
#include <geometry_msgs/Pose.h>
#include <rigidBody.h>
#include <vector>
#include <state_machine.h>
#include <std_msgs/Int32.h>

//This is simulator for Eternity project
class simulator
{
    void init(ros::NodeHandle & nh);
    ros::Subscriber actuator_sub,fc_mode_sub;
    ros::Publisher odometry_pub,acc_pub;

    nav_msgs::Odometry odom;
    dji_sdk::Acceleration acc;

    sensor_msgs::Joy actuators;
    RigidBody rigidBody;
    controller_mode mode;
    int disable_torque;
public:
    ros::NodeHandle & nh;
    simulator(ros::NodeHandle & _nh):nh(_nh)
    {
        init(nh);
    }

    void UpdateActuator(sensor_msgs::Joy joy);

    Eigen::Vector3f local_velocity = Eigen::Vector3f(0,0,0);
    Eigen::Quaternionf attitude = Eigen::Quaternionf(1,0,0,0);
    Eigen::Vector3f angular_velocity;

    void UpdateFcMode(std_msgs::Int32 mode);
    void UpdateTimer(const ros::TimerEvent & timerEvent);
    Eigen::Vector3f air_dynamic_force();
    virtual Eigen::Vector3f CalcForce();
    virtual Eigen::Vector3f CalcTorque();
    Eigen::Vector3f AirdynamicDamping();

    float propellor_force(float n);
    ros::Timer SimulatorTimer;

    //float Cl
    float S = 0.4;
    float d_mass_air = - 0.00;
    //float

    //Ct = a J + b;
    float A_ct = - 0.15;
    float B_ct = 0.09 ;

    float airdynamic_damping_rate_x = 0.02;
    float airdynamic_damping_rate_y = 0.01;
    float airdynamic_damping_rate_z = 0.005;

    float mass;

};
#endif //ETERNITY_FC_SIMULATOR_H
