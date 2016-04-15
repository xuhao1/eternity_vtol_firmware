#ifndef ETERNITY_FC_SERVO_MIXER_CONTROLLER_H
#define ETERNITY_FC_SERVO_MIXER_CONTROLLER_H

#include <dji_sdk/RCChannels.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <state_machine.h>

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
    void update_parameters(ros::NodeHandle & ros);
    void fast_update(const ros::TimerEvent & event);
    void slow_update(const ros::TimerEvent & event);
    void mixer();

    //fix al/ar 1 map to +30deg
    float aileron_angle_ratio;

    //ratio using how much thrust for mz
    float k_thrust_z_ratio;

    static float actuator_rerange(float v,float lowwer,float upper,float lowwer_origin = -1 ,float upper_origin = 1);

    ros::Publisher actucator_control_pub;

    bool arm = false;

    ros::Timer fast_timer;
    ros::Timer slow_timer;

    void mode_sub_callback(const std_msgs::Int32 & mode);

    ros::Subscriber mode_sub;

    controller_mode mode =  controller_mode::nothing;

    ros::Subscriber before_mixer_sub;
    void update_before_mixer(sensor_msgs::Joy joy_data);
    float trim(float v)
    {
        if (v>1)
            return 0.99;
        if (v<-1)
            return -0.99;
        if (isnan(v))
            return 0;
        return v;
    }
};

#endif