#include <state_machine.h>
#include <state_machine_defines.h>
#include <dji_sdk/DroneArmControl.h>

float deltat = 0.005;

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
    if (flight_status == 3 || simulator_arming) {
        action = mode_action::try_arm;
    }
    update_state_machine(action);

}
void state_machine::update_set_points() {
      switch (mode) {
        case controller_mode::hover_attitude : {
            eternity_fc::hover_attitude_sp hover_attitude_sp;
            hover_attitude_sp.roll = rc_value.roll * params.max_attitude_angle /10000;
            hover_attitude_sp.pitch = rc_value.pitch * params.max_attitude_angle / 10000;
            hover_attitude_sp.yaw = rc_value.yaw * params.max_yaw_speed / 10000;
            switch (engine_mode)
            {

                case engine_modes::engine_control_speed:
                hover_attitude_sp.vertical_speed = rc_value.throttle * max_vertical_speed / 10000;
                    break;
                case engine_modes::engine_straight_forward:
                case engine_modes::engine_lock:
                    hover_attitude_sp.vertical_speed = rc_value.throttle / 10000;
                    break;
            }

            hover_attitude_sp.engine_mode = engine_mode;
            hover_attitude_sp.mode = hover_modes ::hover_angular_yaw;
            publishers.hover_att_sp.publish(hover_attitude_sp);
            break;
        };

        case controller_mode::manual :
        {
            eternity_fc::angular_velocity_sp angular_velocity_sp;
            angular_velocity_sp.wx = rc_value.roll * max_angular_velocity / 10000.0f;
            angular_velocity_sp.wy = rc_value.pitch * max_angular_velocity / 10000.0f;
            angular_velocity_sp.wz = rc_value.yaw * max_angular_velocity /10000.0f;
            angular_velocity_sp.throttle = rc_value.throttle / 10000.0f;
            angular_velocity_sp.engine_mode = engine_mode;
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
              rc_possess_pub.publish(angular_velocity_sp);
              break;

          }

    }
}

void state_machine::slow_update(const ros::TimerEvent &event) {
    static int count = 0;
    count ++ ;
    checkRC();
    //change state
    mode_action  action = mode_action::donothing;
    if (rc_value.gear >7000)
    {
        action = mode_action ::toHoverAttitude;
//        action = mode_action ::toManual;
        engine_mode = engine_modes ::engine_straight_forward;
    }
    if(rc_value.gear < 1000 && rc_value.gear > -1000)
    {
        action = mode_action ::toHoverAttitude;
//        action = mode_action ::toManual;
        engine_mode = engine_modes::engine_lock;
    }

    if(rc_value.gear <- 7000) {
        engine_mode = engine_modes::engine_lock;
        try_to_disarm();
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



    ROS_INFO("Mode : %s", controller_mode_names[static_cast<int>(mode)]);
    ROS_INFO("Engine Mode: %s", engine_mode_names[engine_mode]);
    ROS_INFO("RC Value : %f %f %f %f %f", rc_value.roll, rc_value.pitch, rc_value.throttle, rc_value.yaw,
             rc_value.gear);
}

void state_machine::init(ros::NodeHandle &nh) {
    //TODO:is that right?
    rc_channels_sub = nh.subscribe("/dji_sdk/rc_channels",10,&state_machine::update_rc_channels,this);
    joy_sub = nh.subscribe("/joy",10,&state_machine::update_joy,this);
    sdk_permission_sub = nh.subscribe("/dji_sdk/sdk_permission",10,&state_machine::update_control_permission,this);
    flight_status_mode_sub = nh.subscribe("/dji_sdk/flight_status",10,&state_machine::update_flight_status,this);

    angular_velocity_sp_pub = nh.advertise<eternity_fc::angular_velocity_sp>("/setpoints/angular_velocity_sp",10);
    attitude_sp_pub = nh.advertise<eternity_fc::attitude_sp>("attitude_sp",10);
    mode_pub = nh.advertise<std_msgs::Int32>("fc_mode",10);
    rc_possess_pub = nh.advertise<eternity_fc::angular_velocity_sp>("/setpoints/rc_possess",10);

    publishers.hover_att_sp = nh.advertise<eternity_fc::hover_attitude_sp>("/eternity_setpoints/hover_attitude_sp",10);


    nh.param("max_vertical_speed",max_vertical_speed,5.0f);
    nh.param("max_angular_velocity",max_angular_velocity,2.0f);
    nh.getParam("in_simulator",in_simulator);

    nh.param("max_attitude_angle",params.max_attitude_angle,45.0f);
    nh.param("max_yaw_speed",params.max_yaw_speed,180.0f);

    slow_timer = nh.createTimer(ros::Rate(20),&state_machine::slow_update,this);
    fast_timer = nh.createTimer(ros::Rate(50),&state_machine::fast_update,this);

    init_state_machine();


    rc_value.throttle = 0;
    rc_value.gear = 0;
    rc_value.mode = 0;
    rc_value.pitch = 0;
    rc_value.roll = 0;
    rc_value.yaw = 0;

    if (!in_simulator) {
        control_client = nh.serviceClient<dji_sdk::SDKPermissionControl>("/dji_sdk/sdk_permission_control", true);
        drone_arm_client = nh.serviceClient<dji_sdk::DroneArmControl>("/dji_sdk/drone_arm_control",true);
    }




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

    state_transfer[controller_mode::disarm][mode_action::try_arm] = controller_mode::hover_attitude;
    state_transfer[controller_mode::hover_attitude][mode_action::try_disarm] = controller_mode::disarm;
    state_transfer[controller_mode::manual][mode_action::try_disarm] = controller_mode::disarm;
    state_transfer[controller_mode::debug_possess_control][mode_action::try_disarm] = controller_mode::disarm;

    state_transfer[controller_mode::hover_attitude][mode_action::toManual] = controller_mode::manual;
    state_transfer[controller_mode::debug_possess_control][mode_action::toManual] = controller_mode::manual;
    state_transfer[controller_mode::manual][mode_action::toHoverAttitude] = controller_mode::hover_attitude;
    state_transfer[controller_mode::debug_possess_control][mode_action::toHoverAttitude] = controller_mode::hover_attitude;

    state_transfer[controller_mode::hover_attitude][mode_action::toPossess] = controller_mode::debug_possess_control;
    state_transfer[controller_mode::manual][mode_action::toPossess] = controller_mode::debug_possess_control;

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
    this->rc_value = rc_value;
    RCUpdated = true;
}
void state_machine::update_control_permission(std_msgs::UInt8 data) {
    this->obtained_control = data.data > 0;
}

void state_machine::update_joy(sensor_msgs::Joy joy_data) {
    if (joy_data.axes.size() < 4)
        return;
    RCUpdated = true;
    this->rc_value.roll = joy_data.axes[0] * 10000;
    this->rc_value.pitch = joy_data.axes[1] * 10000;
    this->rc_value.throttle = joy_data.axes[2] * 10000;
    this->rc_value.yaw = joy_data.axes[3] * 10000;
    if (joy_data.buttons.size() < 2)
        return;
    this->rc_value.gear = joy_data.buttons[0] * 10000;
    this->simulator_arming = joy_data.buttons[1] == 0;
}

void state_machine::try_to_disarm()
{
    dji_sdk::DroneArmControl con;
    con.request.arm = 0;
    drone_arm_client.call(con);
}

int main(int argc,char ** argv)
{
    ros::init(argc,argv,"state_machine");
    ros::NodeHandle nh("state_machine");
    ROS_INFO("STATE MACHINE Controller READY");

    state_machine ma(nh);

    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}