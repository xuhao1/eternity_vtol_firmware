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
#include <dji_sdk/AttitudeQuaternion.h>
/*
 This is controller for eternity proj
 receieve odometry and accel msg from sensor
 receieve mode attitude_sp angular_velocity_sp from state machine
 */
using namespace Eigen;

inline Vector3f quat2eulers(Quaternionf quat)
{
    Vector3f rpy;
    rpy.x() = atan2f(2 * (quat.w() * quat.x() + quat.y() * quat.z()),1 - 2*(quat.x() * quat.x() + quat.y() * quat.y()));
    rpy.y() = asinf(2 * (quat.w()*quat.y() - quat.z() * quat.x()));
    rpy.z() = atan2f(2* (quat.w () * quat.z() + quat.x() *quat.y()),1-2 * (quat.y() *quat.y() +quat.z() * quat.z() ) );
    return rpy;
}
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

    Eigen::Vector3f  angular_velocity_controller(float DeltaTime,Vector3f angular_vel_sp);
    Eigen::Vector3f so3_attitude_controller(float DeltaTime, Quaternionf attitude_sp,Vector3f external_angular_speed = Vector3f(0,0,0));
    float head_velocity_control(float DeltaTime,float vertical_velocity_sp);

    void run_hover_attitude_controller(float DeltaTime,eternity_fc::hover_attitude_sp hover_sp);
    void run_attitude_controller(float DeltaTime,eternity_fc::attitude_sp attitude_sp,Vector3f external_angular_speed = Vector3f(0,0,0));
    void run_manual_controller(float DelatTime,eternity_fc::angular_velocity_sp angular_velocity_sp1);
//    void run_thrust_controller(float DeltaTime,)


    //states
    Vector3f angular_velocity = Vector3f(0,0,0);
    Quaternionf attitude = Quaternionf(1,0,0,0);
    Vector3f local_velocity = Vector3f(0,0,0);
    Vector3f ground_velocity = Vector3f(0,0,0);
    Vector3f local_accel = Vector3f(0,0,0);
    controller_mode mode =  controller_mode::nothing;
//    nav_msgs::Odometry odometry;

    //params
    Vector3f max_angular_velocity;
    Vector3f angular_velocity_p;
    Vector3f angular_velocity_d;
    Vector3f angular_velocity_i;
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

    struct {
       float max_attitude_angle = 45;
       float max_yaw_speed = 180;
        float k_filter_angular  = 0.1;
        float k_filter_attitude = 0.1;
        float attitude_d_angular_ratio = 0.1;
    } params;

    //Intt values
    float intt_head_velocity_err = 0;

    ros::NodeHandle & nh;


    //setpoints
    eternity_fc::angular_velocity_sp angular_velocity_sp;
    eternity_fc::attitude_sp attitude_sp;
    eternity_fc::hover_attitude_sp hover_attitude_sp;

    bool UsingUnreal;


    //sub
    ros::Subscriber odometry_sub,angular_velocity_sp_sub,attitude_sp_sub,mode_sub,accel_sub;
    ros::Subscriber hover_attitude_sub;
    ros::Subscriber dji_attitude_sub;
    ros::Publisher mixer_pub;
    ros::Publisher angular_setpoints_pub;

    void odometry_callback(const nav_msgs::Odometry & odometry);
    void dji_attitude_callback(const dji_sdk::AttitudeQuaternion & quad);
    void angular_velocity_sp_callback(const eternity_fc::angular_velocity_sp & sp);
    void attitude_sp_callback(const eternity_fc::attitude_sp & sp);
    void mode_sub_callback(const std_msgs::Int32 & mode);
    void accel_sub_callback(const dji_sdk::Acceleration & acc);
    void hover_attitude_sub_callback(const eternity_fc::hover_attitude_sp & sp);
};


#endif //ETERNITY_FC_ATTITUDE_CONTROLLER_H
