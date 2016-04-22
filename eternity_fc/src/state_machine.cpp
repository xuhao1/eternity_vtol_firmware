#include <state_machine.h>

float deltat = 0.01;

void state_machine::fast_update(const ros::TimerEvent &event) {
    //update about control

    checkArm();

    update_set_points();

}
void state_machine::checkRC() {
    if (!RCUpdated)
    {
        noRcUpdateCount ++;
    }
    else
    {
       noRcUpdateCount = 0;
        if (!hasRC)
        {
            hasRC = true;
            ROS_INFO("RC Receive");
        }
    }

    if (noRcUpdateCount > 10)
    {
        static int count_rc_op = 0;
        if (count_rc_op++ %10 == 1)
            ROS_INFO("RC Lost");
    }


    RCUpdated = false;

}
void state_machine::checkArm() {
    mode_action  action = mode_action::try_disarm ;
    if (flight_status == 3) {
        action = mode_action::try_arm;
    }
    update_state_machine(action);

}
void state_machine::update_set_points() {
      switch (mode) {
        case controller_mode::attitude : {
//            Eigen::Quaternionf base_quat ( Eigen::AngleAxisf(M_PI /2, Eigen::Vector3f::UnitY()));
            Eigen::Quaternionf base_quat ( Eigen::AngleAxisf(5*M_PI / 180.0, Eigen::Vector3f::UnitY()));
            float roll_sp = rc_value.roll * max_attitude_angle / 10000.0f;
            float pitch_sp = rc_value.pitch * max_attitude_angle /10000.0f;
            static float yaw_sp = 0;
            yaw_sp += rc_value.yaw * max_yaw_speed * deltat /10000.0f;
            Eigen::AngleAxisf rollAngle(roll_sp * M_PI / 180, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitchAngle(pitch_sp * M_PI / 180, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yawAngle(yaw_sp * M_PI / 180, Eigen::Vector3f::UnitZ());
            Eigen::Quaternionf quat_sp =yawAngle * pitchAngle * rollAngle * base_quat;
            float vertical_speed = rc_value.throttle * max_vertical_speed / 10000.0f;
            eternity_fc::attitude_sp att_sp;
            att_sp.head_speed = vertical_speed;
            att_sp.w = quat_sp.w();
            att_sp.x = quat_sp.x();
            att_sp.y = quat_sp.y();
            att_sp.z = quat_sp.z();
            attitude_sp_pub.publish(att_sp);
            break;
        };

        case controller_mode::manual :
        {
            eternity_fc::angular_velocity_sp angular_velocity_sp;
            angular_velocity_sp.wx = rc_value.roll * max_angular_velocity / 10000.0f;
            angular_velocity_sp.wy = rc_value.pitch * max_angular_velocity / 10000.0f;
            angular_velocity_sp.wz = rc_value.yaw * max_angular_velocity /10000.0f;
            angular_velocity_sp.throttle = rc_value.throttle / 10000.0f;
            angular_velocity_sp_pub.publish(angular_velocity_sp);
            break;
        };

          case debug_possess_control:
          {
              eternity_fc::angular_velocity_sp angular_velocity_sp;
              angular_velocity_sp.wx = rc_value.roll / 10000.0f;
              angular_velocity_sp.wy = rc_value.pitch/ 10000.0f;
              angular_velocity_sp.wz = rc_value.yaw /10000.0f;
              angular_velocity_sp.throttle = rc_value.throttle / 10000.0f;
              angular_velocity_sp_pub.publish(angular_velocity_sp);
              break;

          }

    }
}

void state_machine::slow_update(const ros::TimerEvent &event) {
    checkRC();
    //change state
    mode_action  action = mode_action::donothing;
    if (rc_value.gear <-7000)
    {
         action = mode_action::toManual;
    }
    if(rc_value.gear < 1000 && rc_value.gear > -1000)
    {
        action = mode_action::toAttitude;
    }
    if(rc_value.gear > 7000) {
        action = mode_action::toPossess;
    }

    update_state_machine(action);

    std_msgs::Int32 mode_tmp;
    mode_tmp.data = mode;
    mode_pub.publish(mode_tmp);

    if (!in_simulator && control_client)
    {
        dji_sdk::SDKPermissionControl req;
        req.request.control_enable = true;
        control_client.call(req);
    }

}

void state_machine::init(ros::NodeHandle &nh) {
    //TODO:is that right?
    rc_channels_sub = nh.subscribe("/dji_sdk/rc_channels",10,&state_machine::update_rc_channels,this);
    joy_sub = nh.subscribe("/joy",10,&state_machine::update_joy,this);
    sdk_permission_sub = nh.subscribe("/dji_sdk/sdk_permission",10,&state_machine::update_control_permission,this);
    flight_status_mode_sub = nh.subscribe("/dji_sdk/flight_status",10,&state_machine::update_flight_status,this);

    angular_velocity_sp_pub = nh.advertise<eternity_fc::angular_velocity_sp>("angular_velocity_sp",10);
    attitude_sp_pub = nh.advertise<eternity_fc::attitude_sp>("attitude_sp",10);
    mode_pub = nh.advertise<std_msgs::Int32>("fc_mode",10);


    nh.param("max_attitude_angle",max_attitude_angle,45.0f);
    nh.param("max_yaw_speed",max_yaw_speed,180.0f);
    nh.param("max_vertical_speed",max_vertical_speed,5.0f);
    nh.param("max_angular_velocity",max_angular_velocity,2.0f);
    nh.param("using_rc_or_joy",using_rc_or_joy,false);
    nh.getParam("in_simulator",in_simulator);

    slow_timer = nh.createTimer(ros::Rate(10),&state_machine::slow_update,this);
    fast_timer = nh.createTimer(ros::Rate(100),&state_machine::fast_update,this);

    init_state_machine();


    rc_value.throttle = 0;
    rc_value.gear = 0;
    rc_value.mode = 0;
    rc_value.pitch = 0;
    rc_value.roll = 0;
    rc_value.yaw = 0;

    if (!in_simulator)
        control_client = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control",true);


}

void state_machine::update_flight_status(std_msgs::UInt8 data) {
    flight_status = data.data;
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

    state_transfer[controller_mode::disarm][mode_action::try_arm] = controller_mode::attitude;
    state_transfer[controller_mode::attitude][mode_action::try_disarm] = controller_mode::disarm;
    state_transfer[controller_mode::manual][mode_action::try_disarm] = controller_mode::disarm;
    state_transfer[controller_mode::debug_possess_control][mode_action::try_disarm] = controller_mode::disarm;

    state_transfer[controller_mode::attitude][mode_action::toManual] = controller_mode::manual;
    state_transfer[controller_mode::debug_possess_control][mode_action::toManual] = controller_mode::manual;
    state_transfer[controller_mode::manual][mode_action::toAttitude] = controller_mode::attitude;
    state_transfer[controller_mode::debug_possess_control][mode_action::toAttitude] = controller_mode::attitude;

    state_transfer[controller_mode::attitude][mode_action::toPossess] = controller_mode::debug_possess_control;
    state_transfer[controller_mode::manual][mode_action::toPossess] = controller_mode::debug_possess_control;

    mode = controller_mode ::attitude;
}

void state_machine::update_state_machine(mode_action act) {
    controller_mode  mode_tmp = state_transfer[mode][act];
    if (mode_tmp != controller_mode::nothing)
    {
        if (mode_tmp != mode)
        {
            switch (mode_tmp)
            {
                case controller_mode ::disarm:
                    ROS_INFO("Disarm!!!");
                    break;
                case controller_mode ::attitude:
                    ROS_INFO("Attitude!!!");
                    break;
                case controller_mode ::manual:
                    ROS_INFO("Manual!!!");
                    break;
                case controller_mode ::debug_possess_control:
                    ROS_INFO("Possess Control!!!");
                    break;
                default:
                    ROS_INFO("Unknow State %d",mode_tmp);
            }
        }
        mode = mode_tmp;
    }
}

void state_machine::update_rc_channels(RCChannels rc_value) {
    //TODO:Use real delta time
    if (!using_rc_or_joy) {
        this->rc_value = rc_value;
        RCUpdated = true;
    }
}
void state_machine::update_control_permission(std_msgs::UInt8 data) {
//    ROS_INFO("control :%d",data.data);
    this->obtained_control = data.data > 0;
}

void state_machine::update_joy(sensor_msgs::Joy joy_data) {
    if (using_rc_or_joy)
    {
        if (joy_data.axes.size() < 4)
            return;
        RCUpdated = true;
        this->rc_value.roll = joy_data.axes[0] * 10000;
        this->rc_value.pitch = joy_data.axes[1] * 10000;
        this->rc_value.throttle = joy_data.axes[2] * 10000;
        this->rc_value.yaw = joy_data.axes[3] * 10000;
        if (joy_data.buttons.size() < 1)
            return;
        this->rc_value.mode = joy_data.buttons[0] * 10000;
    }
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