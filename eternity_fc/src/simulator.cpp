#include "simulator.h"
#define deltatime 0.001
using namespace Eigen;

inline float Power(float a, float b)
{
    return pow(a, b);
}

void simulator::init(ros::NodeHandle & nh)
{

    nh.param("disable_torque",disable_torque,0);
    nh.getParam("mass",mass);
    nh.param("simulator/d_mass_air",d_mass_air,-0.1f);
    nh.param("simulator/airdynamic_damping_rate_x",airdynamic_damping_rate_x,0.1f);
    nh.param("simulator/airdynamic_damping_rate_y",airdynamic_damping_rate_y,0.05f);
    nh.param("simulator/airdynamic_damping_rate_z",airdynamic_damping_rate_z,0.01f);

    actuator_sub = nh.subscribe("/dji_sdk/possess_control",10,&simulator::UpdateActuator,this);


    fc_mode_sub = nh.subscribe("/state_machine/fc_mode",10,&simulator::UpdateFcMode,this);

    odometry_pub = nh.advertise<nav_msgs::Odometry>("/dji_sdk/odometry",10);
    acc_pub = nh.advertise<dji_sdk::Acceleration>("/dji_sdk/acceleration",10);

    actuators.axes = std::vector<float>(8);

    for (int i = 0;i<8;i++)
        actuators.axes[i] = 0;

    SimulatorTimer = nh.createTimer(ros::Rate(1/deltatime),&simulator::UpdateTimer,this);

    mode = controller_mode ::nothing;
//    ROS_INFO("disable torque : %d",disable_torque);
    ROS_INFO("Mass %f",mass);
    ROS_INFO("d mass air %5f",d_mass_air);

    this->rigidBody.mass = mass;

    //w x y z 0.707 0 -0.707
    Eigen::Quaterniond base_quat ( Eigen::AngleAxisd(M_PI /2, Eigen::Vector3d::UnitY()));
}


void simulator::UpdateActuator(sensor_msgs::Joy joy) {
    for (int i = 0;i<joy.axes.size();i++)
    {
        joy.axes[i] = (joy.axes[i] /100 - 0.5)*2;
    }
    this->actuators = joy;
}

void simulator::UpdateFcMode(std_msgs::Int32 mode) {
    this->mode = static_cast<controller_mode>(mode.data);
}

void simulator::UpdateTimer(const ros::TimerEvent & timerEvent) {
    static int count = 0;
    count ++;

    Vector3f pos = rigidBody.get_position().cast<float>();
    Vector3f vel = rigidBody.get_velocity().cast<float>();
    angular_velocity = rigidBody.get_angularVelocity().cast<float>();

    attitude = rigidBody.get_attitude().cast<float>();
    local_velocity = attitude.inverse()._transformVector(rigidBody.get_velocity().cast<float>());

    if (count % 10 == 0) {
        odom.pose.pose.position.x = pos.x();
        odom.pose.pose.position.y = pos.y();
        odom.pose.pose.position.z = -pos.z();

        odom.pose.pose.orientation.w = attitude.w();
        odom.pose.pose.orientation.x = attitude.x();
        odom.pose.pose.orientation.y = attitude.y();
        odom.pose.pose.orientation.z = attitude.z();

        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = -vel.z();

        odom.twist.twist.angular.x = angular_velocity.x();
        odom.twist.twist.angular.y = angular_velocity.y();
        odom.twist.twist.angular.z = angular_velocity.z();

        odometry_pub.publish(odom);
        //acc_pub.publish(acc_pub);
    }

    Vector3d force = CalcForce().cast<double>();
    Vector3d torque = CalcTorque().cast<double>();
//    torque.z() = 0;
//    torque.y() = 0;
    rigidBody.set_force(force.cast<double>());
    rigidBody.set_torque(torque.cast<double>());
    rigidBody.sim_step(deltatime);
}

Eigen::Vector3f simulator::CalcForce() {
    static int count = 0;
    count ++;

    float LeftMotor = (actuators.axes[2] + 1) /2.0 * 177.6;
    float RightMotor = (actuators.axes[3] + 1) /2.0 * 177.6;
    Vector3f force_origin = Vector3f(propellor_force(LeftMotor)+propellor_force(RightMotor),0,0);
    Vector3f gravity = Vector3f(0,0,9.81) * this->rigidBody.get_mass();

    if (mode <= controller_mode::disarm)
    {
        force_origin = Vector3f(0,0,0);
        gravity = Vector3f(0,0,0);
    }
    Vector3f Force = attitude._transformVector(force_origin + air_dynamic_force()) + gravity;
    return Force;
}

Eigen::Vector3f simulator::air_dynamic_force() {
    static int count = 0;
    count ++;
    Vector3f airdynamic_force(0,0,0);

    Vector3f vel = rigidBody.get_velocity().cast<float>();
    air_relative_velocity = attitude.inverse()._transformVector(vel-wind_speed);


    if (air_relative_velocity.norm()>0.1)
    {
        float v = air_relative_velocity.norm();
        float a = - atan2f(air_relative_velocity.z(),air_relative_velocity.x());

        //0.087 is angle for 5 deg,this is for origin fraction
        float drag = (2*sin(a)*sin(a) + 2*0.087*0.087)* 0.5 *S * v*v;
        float lift = sin(2*a) * 0.5*S *v *v;

        //Drag force
        airdynamic_force = - air_relative_velocity.normalized() * drag;

        //Lift vector
//        Vector3f lift3f(-air_relative_velocity.z(),0,air_relative_velocity.x());
        Vector3f lift3f(0,0,-1);
        lift3f = lift3f.normalized() * lift;
        airdynamic_force = airdynamic_force + lift3f;

    }
    return airdynamic_force;

}

float simulator::propellor_force(float n) {
    static int count = 0;
    count ++;
    float J = local_velocity.x()/(n * 0.254);
    float ct = A_ct * J + B_ct;
    if (n<0.01 || local_velocity.x() < 0)
    {
        ct = 0.09;
    }
    float T = ct * 1.29 * Power(0.254,4) *  n * n;

    if (count % 10 ==0 )
    {
//        ROS_INFO("v :%f Ct %f T %f",local_velocity.x(),ct,T);
    }
    return T;
}
Eigen::Vector3f simulator::AirdynamicDamping() {
    float damping_x = - airdynamic_damping_rate_x * angular_velocity.y() * air_relative_velocity.y() * air_relative_velocity.y() * 0.5 * 1.29;
    float damping_y = - airdynamic_damping_rate_y * angular_velocity.y() * air_relative_velocity.x() * air_relative_velocity.x() * 0.5 * 1.29;
    float damping_z = - airdynamic_damping_rate_z * angular_velocity.z() * air_relative_velocity.x() * air_relative_velocity.x() * 0.5 * 1.29;
    return Eigen::Vector3f(damping_x,damping_y,damping_z);
}
Eigen::Vector3f simulator::CalcTorque() {
    float LeftWing = - actuators.axes[0];
    float RightWing = actuators.axes[1];
    float LeftMotor = (actuators.axes[2] + 1) /2.0 * 177.6;
    float RightMotor = (actuators.axes[3] + 1) /2.0 * 177.6;

    float al = LeftWing * M_PI / 6;
    float ar = RightWing * M_PI / 6;
    float nl = LeftMotor;
    float nr = RightMotor;
    Vector3f M;
    M << (0.00002690160416922408 + 0.00008260145139059342*al)*Power(nl, 2) + (-0.00002690160416922408 - 0.00008260145139059342*ar)*Power(nr, 2), -0.00004688190484330978*al*Power(nl, 2) - 0.00004688190484330978*ar*Power(nr, 2), 0.00015186361805013624*Power(nl, 2) - 0.00015186361805013624*Power(nr, 2);
    Vector3f airdyamic_center(d_mass_air,0,0);
    Vector3f air_dynamic_torque = airdyamic_center.cross(air_dynamic_force()) + AirdynamicDamping();
    return M + air_dynamic_torque;
}

int main(int argc,char ** argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh;
    simulator sim(nh);
    ROS_INFO("SIMULATOR READY!!!");
    ros::AsyncSpinner spinner(2); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
}
