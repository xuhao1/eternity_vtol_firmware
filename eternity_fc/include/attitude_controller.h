//
// Created by xuhao on 2016/4/5.
//

#ifndef ETERNITY_FC_ATTITUDE_CONTROLLER_H
#define ETERNITY_FC_ATTITUDE_CONTROLLER_H

#include <eternity_fc/angular_velocity_sp.h>
#include <eternity_fc/attitude_sp.h>
#include <eigen3/Eigen/Eigen>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <state_machine.h>
#include <dji_sdk/Acceleration.h>
/*
 This is controller for eternity proj
 receieve odometry and accel msg from sensor
 receieve mode attitude_sp angular_velocity_sp from state machine
 */
using namespace Eigen;

class attitude_controller
{
public:
    void init(ros::NodeHandle & nh);
    void init_sub(ros::NodeHandle & nh);
    //update 100hz
    void fast_update(const ros::TimerEvent & timerEvent);
    void slow_update(const ros::TimerEvent & timerEvent);
    void run_controller(const ros::TimerEvent & timerEvent);
    void update_params(ros::NodeHandle & nh);

    ros::Timer fast_timer;
    ros::Timer slow_timer;

    attitude_controller(ros::NodeHandle & _nh):
            nh(_nh)
    {
        init(_nh);
    }


    static void angle_axis_from_quat(Quaternionf q0, Quaternionf q_sp, Vector3f & axis);

    void angular_velocity_controller(float DeltaTime,Vector3f angular_vel_sp);
    void so3_attitude_controller(float DeltaTime, Quaternionf attitude_sp);
    void head_velocity_control(float DeltaTime,float vertical_velocity_sp);

    //states
    Vector3f angular_velocity = Vector3f(0,0,0);
    Quaternionf attitude = Quaternionf(1,0,0,0);
    Vector3f local_velocity = Vector3f(0,0,0);
    Vector3f ground_velocity = Vector3f(0,0,0);
    Vector3f local_accel = Vector3f(0,0,0);
    controller_mode mode =  controller_mode::nothing;
    nav_msgs::Odometry odometry;

    //params
    Vector3f max_angular_velocity;
    Vector3f angular_velocity_p;
    Vector3f angular_velocity_d;
    Vector3f attitude_p;
    Vector3f attitude_i;
    Vector3f attitude_d;
    float head_velocity_i;
    float head_velocity_p;
    float head_velocity_d;
    float thrust_weight_ratio;

    //output
    float Aileron = 0;
    float Elevator = 0;
    float Rudder = 0;
    float Thrust = 0;

    //Intt values
    float intt_head_velocity_err = 0;

    ros::NodeHandle & nh;


    //setpoints
    eternity_fc::angular_velocity_sp angular_velocity_sp;
    eternity_fc::attitude_sp attitude_sp;

    bool UsingUnreal;


    //sub
    ros::Subscriber odometry_sub,angular_velocity_sp_sub,attitude_sp_sub,mode_sub,accel_sub;
    ros::Publisher mixer_pub;

    void odometry_callback(const nav_msgs::Odometry & odometry);
    void angular_velocity_callback(const eternity_fc::angular_velocity_sp & sp);
    void attitude_sp_callback(const eternity_fc::attitude_sp & sp);
    void mode_sub_callback(const std_msgs::Int32 & mode);
    void accel_sub_callback(const dji_sdk::Acceleration & acc);
};


#endif //ETERNITY_FC_ATTITUDE_CONTROLLER_H
