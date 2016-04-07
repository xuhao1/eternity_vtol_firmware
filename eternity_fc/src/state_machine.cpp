#include <state_machine.h>

float deltat = 0.01;

void state_machine::fast_update(const ros::TimerEvent &event) {
    //update about control
    mode_action  action = mode_action::donothing ;
    static int rc_arm_tick = 0;
    if (rc_value.throttle < -9500 && rc_value.pitch < -9500 && rc_value.yaw > 9500 && rc_value.roll < -9500)
    {
        rc_trying_arm = true;
    }
    if (rc_trying_arm)
    {
        rc_arm_tick ++;
    } else {
        rc_arm_tick = 0;
    }

    if (rc_arm_tick > 100)
    {
        rc_arm_tick = 0;
        action = mode_action::arm;
    }

    update_state_machine(action);

    std_msgs::Int32 mode_tmp;
    mode_tmp.data = mode;
    mode_pub.publish(mode_tmp);

    update_set_points();

}

void state_machine::update_set_points() {
      switch (mode) {
        case controller_mode::attitude : {
            float roll_sp = rc_value.roll * max_attitude_angle / 10000.0f;
            float pitch_sp = rc_value.pitch * max_attitude_angle /10000.0f;
            static float yaw_sp = 0;
            yaw_sp += rc_value.yaw * max_yaw_speed * deltat /10000.0f;
            Eigen::AngleAxisd rollAngle(roll_sp * M_PI / 180, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch_sp * M_PI / 180, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw_sp * M_PI / 180, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_sp = yawAngle * pitchAngle * rollAngle;
            float vertical_speed = rc_value.throttle * max_vertical_speed;
            eternity_fc::attitude_sp att_sp;
            att_sp.head_speed = vertical_speed;
            att_sp.w = quat_sp.w();
            att_sp.x = quat_sp.x();
            att_sp.y = quat_sp.y();
            att_sp.z = quat_sp.z();
            attitude_sp_pub.publish(att_sp);
            break;
        }

        case controller_mode::manual : {
            eternity_fc::angular_velocity_sp angular_velocity_sp;
            angular_velocity_sp.wx = rc_value.roll * max_angular_velocity / 10000.0f;
            angular_velocity_sp.wy = rc_value.pitch * max_angular_velocity / 10000.0f;
            angular_velocity_sp.wz = rc_value.yaw * max_angular_velocity /10000.0f;
            angular_velocity_sp.throttle = rc_value.throttle / 10000.0f;
            angular_velocity_sp_pub.publish(angular_velocity_sp);
            break;
        }
    }
}

void state_machine::slow_update(const ros::TimerEvent &event) {
    //change state
    mode_action  action = mode_action::donothing;
    if (rc_value.mode <-9500)
    {
         action = mode_action::toManual;
    }
    if(rc_value.mode > 9500)
    {
        action = mode_action::toAttitude;
    }

    update_state_machine(action);

}

void state_machine::init(ros::NodeHandle &nh) {
    //TODO:is that right?
    rc_channels_sub = nh.subscribe("/dji_sdk/rc_channels",10,&state_machine::update_rc_channels,this);
    angular_velocity_sp_pub = nh.advertise<eternity_fc::angular_velocity_sp>("angular_velocity_sp",10);
    attitude_sp_pub = nh.advertise<eternity_fc::attitude_sp>("attitude_sp",10);
    mode_pub = nh.advertise<std_msgs::Int32>("fc_mode",10);


    nh.param("max_attitude_angle",max_attitude_angle,45.0f);
    nh.param("max_yaw_speed",max_yaw_speed,180.0f);
    nh.param("max_vertical_speed",max_vertical_speed,5.0f);
    nh.param("max_angular_velocity",max_angular_velocity,1.0f);

    slow_timer = nh.createTimer(ros::Rate(10),&state_machine::slow_update,this);
    fast_timer = nh.createTimer(ros::Rate(100),&state_machine::fast_update,this);

    init_state_machine();


    rc_value.throttle = 0;
    rc_value.gear = 0;
    rc_value.mode = 0;
    rc_value.pitch = 0;
    rc_value.roll = 0;
    rc_value.yaw = 0;
}

void state_machine::init_state_machine() {
    mode = controller_mode::disarm;

    for (int i = 0 ; i < controller_mode::mode_end ; i++)
    {
        for (int j = 0 ; j < mode_action::action_end; j++)
        {
            state_transfer[static_cast<controller_mode>(i)][static_cast<mode_action >(j)] = controller_mode::nothing;
        }
    }

    state_transfer[controller_mode::disarm][mode_action::arm] = controller_mode::attitude;
    state_transfer[controller_mode::attitude][mode_action::arm] = controller_mode::disarm;
    state_transfer[controller_mode::manual][mode_action::arm] = controller_mode::disarm;

    state_transfer[controller_mode::attitude][mode_action::toManual] = controller_mode::manual;
    state_transfer[controller_mode::manual][mode_action::toAttitude] = controller_mode::attitude;

    //For debug!!! Attetion
    mode = controller_mode ::attitude;
}

void state_machine::update_state_machine(mode_action act) {
    controller_mode  mode_tmp = state_transfer[mode][act];
    if (mode_tmp != controller_mode::nothing)
    {
        mode = mode_tmp;
    }
}

void state_machine::update_rc_channels(RCChannels rc_value) {
    //TODO:Use real delta time
    this->rc_value = rc_value;
}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"state_machine");
    ros::NodeHandle nh("state_machine");
    ROS_INFO("STATE MACHINE Controller READY");

    state_machine ma(nh);

    ros::spin();
    return 0;
}