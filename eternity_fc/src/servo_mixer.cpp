//
// Created by xuhao on 2016/4/5.
//

#include <servo_mixer.h>

#define EASY_MIXER

inline float Power(float a, float b)
{
    return pow(a, b);
}

inline float Sqrt(float a)
{
    return sqrt(a);
}

void servo_mixer::update_parameters(ros::NodeHandle &nh)
{
    nh.param("aileron_angle_ratio", aileron_angle_ratio, 0.5f);
    nh.param("k_thrust_z_ratio", k_thrust_z_ratio, 0.15f);
    nh.param("torque_middle_point_x", torque_middle_point.x(), 0.0f);
    nh.param("torque_middle_point_y", torque_middle_point.y(), 0.0f);
    nh.param("torque_middle_point_z", torque_middle_point.z(), 0.0f);
    nh.param("manual_amplitude_x", manual_amplitude.x(), 1.0f);
    nh.param("manual_amplitude_y", manual_amplitude.y(), 1.0f);
    nh.param("manual_amplitude_z", manual_amplitude.z(), 1.0f);
    nh.param("mixer_rate",mixer_rate,50);
//    ROS_INFO("torque_middle_point_x %5f",torque_middle_point.x());
}

void servo_mixer::init(ros::NodeHandle &nh)
{
    actucator_control_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/possess_control", 10);


    before_mixer.axes = std::vector<float>(8);
    after_mixer.axes = std::vector<float>(8);
    for (int i = 0; i < 8; i++) {
        before_mixer.axes[i] = 0;
        after_mixer.axes[i] = 0;
    }

    update_parameters(nh);

    mode_sub = nh.subscribe("/state_machine/fc_mode", 10, &servo_mixer::mode_sub_callback, this);
    before_mixer_sub = nh.subscribe("/attitude_controller/mixer", 10, &servo_mixer::update_before_mixer, this);
    rc_possess_sub = nh.subscribe("/setpoints/rc_possess", 10, &servo_mixer::update_rc_values, this);

    fast_timer = nh.createTimer(ros::Rate(mixer_rate), &servo_mixer::fast_update, this);
    slow_timer = nh.createTimer(ros::Rate(10), &servo_mixer::slow_update, this);

}

void servo_mixer::mode_sub_callback(const std_msgs::Int32 &mode)
{
    this->mode = static_cast<controller_mode>(mode.data);

    //TODO::more complex
    if (this->mode > controller_mode::disarm && this->mode < controller_mode::mode_end) {
        this->arm = true;
    }
    else {
        this->arm = false;
    }
}

inline float rerange_servo(float v)
{
    if (v > 1)
        return 1.001;
    if (v < -1)
        return -1.001;
    return 0;
}

void servo_mixer::update_rc_values(eternity_fc::angular_velocity_sp data)
{
    this->rc_posses_data = data;
}

Eigen::Vector4f servo_mixer::calcuateActuators(Eigen::Vector3f u_xyz, float Thrust)
{
    float al = 0, ar = 0, ul = 0, ur = 0;
    //u 2 actuator
    //mix
    al = (u_xyz.x() - u_xyz.y());
    ar = (-u_xyz.x() - u_xyz.y());


    al = actuator_rerange(al, 50 * (1 + aileron_angle_ratio), 50 * (1 - aileron_angle_ratio));
    ar = actuator_rerange(ar, 50 * (1 - aileron_angle_ratio), 50 * (1 + aileron_angle_ratio));
    //ul from 0 to 1
    if (Thrust + u_xyz.z() * k_thrust_z_ratio > 0 && Thrust > 0.01)
        ul = sqrt(Thrust + u_xyz.z() * k_thrust_z_ratio);
    else
        ul = 0;

    //ur from 0 to 1
    if (Thrust - u_xyz.z() * k_thrust_z_ratio > 0 && Thrust > 0.01)
        ur = sqrt(Thrust - u_xyz.z() * k_thrust_z_ratio);
    else
        ur = 0;

    ul = actuator_rerange(ul, 0, 100, 0, 1);
    ur = actuator_rerange(ur, 0, 100, 0, 1);

    Eigen::Vector4f res;
    res(0) = al;
    res(1) = ar;
    res(2) = ul;
    res(3) = ur;
    return res;
}

void servo_mixer::calcuateU(Eigen::Vector3f &u_xyz, float &Ut)
{
#ifdef  EASY_MIXER
    u_xyz.x() = (before_mixer.axes[0] + torque_middle_point.x()) * manual_amplitude.x();
    u_xyz.y() = (before_mixer.axes[1] + torque_middle_point.y()) * manual_amplitude.y();
    u_xyz.z() = (before_mixer.axes[3] + torque_middle_point.z()) * manual_amplitude.z();
    Ut = before_mixer.axes[2];
#else
    float Mx = before_mixer.axes[0] + torque_middle_point.x();
    float My = before_mixer.axes[1] + torque_middle_point.y();
    float Mz = before_mixer.axes[3] + torque_middle_point.z();
    float Thrust = before_mixer.axes[2];

    if (Thrust > 0.3) {
        u_xyz <<
        (-1.953624230872906e5 * My * Mz - 1.0622537302872798e6 * Mx * Thrust + 1.881710033408788e5 * Mz * Thrust) /
        (3.1578475658491784e4 * Power(Mz, 2) - 2.898214227537045e6 * Power(Thrust, 2)),
                (-1.1088137526575953e5 * Mx * Mz + 1.964187937464974e4 * Power(Mz, 2) -
                 1.8715899057442548e6 * My * Thrust) /
                (3.1578475658491784e4 * Power(Mz, 2) - 2.898214227537045e6 * Power(Thrust, 2)), 0.6958875712665018 *
                                                                                                Mz;
    }
    else {
        float Thrust = 0.3;
        u_xyz <<
        (-1.953624230872906e5 * My * Mz - 1.0622537302872798e6 * Mx * Thrust + 1.881710033408788e5 * Mz * Thrust) /
        (3.1578475658491784e4 * Power(Mz, 2) - 2.898214227537045e6 * Power(Thrust, 2)),
                (-1.1088137526575953e5 * Mx * Mz + 1.964187937464974e4 * Power(Mz, 2) -
                 1.8715899057442548e6 * My * Thrust) /
                (3.1578475658491784e4 * Power(Mz, 2) - 2.898214227537045e6 * Power(Thrust, 2)), 0.6958875712665018 *
                                                                                                Mz;
    }
    Ut = Thrust;
#endif

}

void servo_mixer::mixer()
{
    if (this->mode == controller_mode::debug_possess_control) {
        after_mixer.axes[0] = actuator_rerange(rc_posses_data.wx);
        after_mixer.axes[1] = actuator_rerange(rc_posses_data.wy);
        after_mixer.axes[2] = actuator_rerange(rc_posses_data.throttle);
        after_mixer.axes[3] = actuator_rerange(rc_posses_data.wz);
    }
    else {
        Eigen::Vector3f u_xyz;
        float Thrust;
        calcuateU(u_xyz, Thrust);
        Eigen::Vector4f actuators = calcuateActuators(u_xyz, Thrust);

        after_mixer.axes[0] = actuators(0);
        after_mixer.axes[1] = actuators(1);

        if (this->arm) {
            after_mixer.axes[2] = actuators(2);
            after_mixer.axes[3] = actuators(3);
        }
        else {
            after_mixer.axes[2] = 0;
            after_mixer.axes[3] = 0;
        }
    }

    //Trim

    for (int i = 0; i < 8; i++) {
        if (isnan(after_mixer.axes[i])) {
            after_mixer.axes[i] = 0;
        }
        else
            after_mixer.axes[i] = trim(after_mixer.axes[i]);
    }
    actucator_control_pub.publish(after_mixer);
}

float servo_mixer::actuator_rerange(float v, float lowwer, float upper, float lowwer_origin, float upper_origin)
{
    float tmp = (v - lowwer_origin) / (upper_origin - lowwer_origin);
    return lowwer + tmp * (upper - lowwer);
}

void servo_mixer::fast_update(const ros::TimerEvent &event)
{
    mixer();
}

void servo_mixer::slow_update(const ros::TimerEvent &event)
{
    update_parameters(nh);
}

void servo_mixer::update_before_mixer(sensor_msgs::Joy joy_data)
{
    this->before_mixer = joy_data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_mixer");
    ros::NodeHandle nh("servo_mixer");
    servo_mixer sm(nh);

    ROS_INFO("SERVO MIXER Controller READY");
    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
}

