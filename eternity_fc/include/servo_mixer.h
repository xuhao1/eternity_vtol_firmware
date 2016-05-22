#ifndef ETERNITY_FC_SERVO_MIXER_CONTROLLER_H
#define ETERNITY_FC_SERVO_MIXER_CONTROLLER_H

#include <dji_sdk/RCChannels.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <state_machine.h>
#include <eternity_fc/angular_velocity_sp.h>

class servo_mixer
{
public:
    ros::NodeHandle &nh;
    servo_mixer(ros::NodeHandle & _nh):
            nh(_nh){
        init(nh);
    }
    sensor_msgs::Joy before_mixer;

    sensor_msgs::Joy after_mixer;

    void init(ros::NodeHandle & nh);
    void update_parameters(ros::NodeHandle & nh);
    void fast_update(const ros::TimerEvent & event);
    void slow_update(const ros::TimerEvent & event);
    void mixer();

    //fix al/ar 1 map to +30deg
    float aileron_angle_ratio;

    //ratio using how much thrust for mz
    float k_thrust_z_ratio;

    static float actuator_rerange(float v,float lowwer = 0,float upper = 100,float lowwer_origin = -1 ,float upper_origin = 1);

    ros::Publisher actucator_control_pub;

    bool arm = false;
    eternity_fc::angular_velocity_sp rc_posses_data;

    Eigen::Vector3f torque_middle_point = Eigen::Vector3f(0,0,0);

    ros::Timer fast_timer;
    ros::Timer slow_timer;

    Eigen::Vector3f manual_amplitude;

    void mode_sub_callback(const std_msgs::Int32 & mode);

    ros::Subscriber mode_sub;
    ros::Subscriber rc_possess_sub;

    controller_mode mode =  controller_mode::nothing;

    ros::Subscriber before_mixer_sub;
    void update_before_mixer(sensor_msgs::Joy joy_data);
    void update_rc_values(eternity_fc::angular_velocity_sp data);
    void calcuateU(Eigen::Vector3f& u_xyz,float & Ut);
    Eigen::Vector4f calcuateActuators(Eigen::Vector3f u_xyz,float Thrust);
    float trim(float v)
    {
        if (v>100)
            return 100;
        if (v<0)
            return 0;
        if (isnan(v))
            return 50;
        return v;
    }
};

#endif