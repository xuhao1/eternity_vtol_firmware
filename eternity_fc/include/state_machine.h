//
// Created by xuhao on 2016/4/5.
//

#ifndef ETERNITY_FC_STATE_MACHINE_H
#define ETERNITY_FC_STATE_MACHINE_H

#include <dji_sdk/RCChannels.h>
#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <eternity_fc/attitude_sp.h>
#include <eternity_fc/angular_velocity_sp.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/Joy.h>
#include <dji_sdk/SDKPermissionControl.h>

using namespace dji_sdk;

enum controller_mode {
    nothing = 0,
    disarm = 1,
    attitude = 2,
    manual = 3,
    debug_possess_control = 4,
    mode_end
};

enum mode_action {
    donothing = 0,
    arm = 1,
    toManual = 2,
    toAttitude = 3,
    action_end
};

//receive joy or rc
class state_machine
{
public:
    void init(ros::NodeHandle & nh);
    virtual void fast_update(const ros::TimerEvent& event);
    virtual void slow_update(const ros::TimerEvent& event);

    ros::Subscriber rc_channels_sub;
    ros::Subscriber joy_sub;
    ros::Subscriber sdk_permission_sub;
    ros::ServiceClient control_client;

    void update_rc_channels(RCChannels rc_value);
    void update_joy(sensor_msgs::Joy joy_data);
    RCChannels rc_value;
    //Run 100hz
    ros::Timer fast_timer;
    //Run 10hz
    ros::Timer slow_timer;

    ros::Publisher angular_velocity_sp_pub;
    ros::Publisher attitude_sp_pub;
    ros::Publisher mode_pub;

    controller_mode mode;

    float max_attitude_angle;
    float max_yaw_speed;
    float max_vertical_speed;
    float max_angular_velocity;
    bool rc_trying_arm = false;
    state_machine (ros::NodeHandle & _nh):
            nh(_nh)
    {
        init(_nh);
    }

    controller_mode state_transfer[10][10];

    void update_state_machine(mode_action act);
    void update_set_points();
    void update_control_permission(std_msgs::UInt8 data);

    void init_state_machine();

    bool hasRC = false;

    bool RCUpdated = false;

    int noRcUpdateCount = 0;

    bool obtained_control = false;
    ros::NodeHandle & nh;

    void checkArm();
    void checkRC();
    //false:rc
    //true joy
    bool using_rc_or_joy;

};

#endif //ETERNITY_FC_STATE_MACHINE_H
