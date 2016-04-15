#include "simulator.h"
#define deltatime 0.01
using namespace Eigen;

inline float Power(float a, float b)
{
    return pow(a, b);
}

void simulator::init(ros::NodeHandle & nh)
{

    nh.param("disable_torque",disable_torque,0);
    nh.getParam("mass",mass);

    actuator_sub = nh.subscribe("/servo_mixer/actuators",10,&simulator::UpdateActuator,this);


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

    this->rigidBody.mass = mass;
}


void simulator::UpdateActuator(sensor_msgs::Joy joy) {
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

    odom.pose.pose.position.x = pos.x();
    odom.pose.pose.position.y = pos.y();
    odom.pose.pose.position.z = - pos.z();

    odom.pose.pose.orientation.w = attitude.w();
    odom.pose.pose.orientation.x = attitude.x();
    odom.pose.pose.orientation.y = attitude.y();
    odom.pose.pose.orientation.z = attitude.z();

    odom.twist.twist.linear.x = vel.x();
    odom.twist.twist.linear.y = vel.y();
    odom.twist.twist.linear.z = - vel.z();

    odom.twist.twist.angular.x = angular_velocity.x();
    odom.twist.twist.angular.y = angular_velocity.y();
    odom.twist.twist.angular.z = angular_velocity.z();

    odometry_pub.publish(odom);
    //acc_pub.publish(acc_pub);

    Vector3d force = CalcForce().cast<double>();
    Vector3d torque = CalcTorque().cast<double>();
    rigidBody.set_force(force.cast<double>());
    rigidBody.set_torque(torque.cast<double>());
    rigidBody.sim_step(deltatime);

    if (count % 10 ==0)
    {
        //ROS_INFO("time %f",((float)count) * deltatime);
    }
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
    if (local_velocity.norm()>0.1)
    {
        float v = local_velocity.norm();
        float a = - atan2f(local_velocity.z(),local_velocity.x());

        //0.087 is angle for 5 deg,this is for origin fraction
        float drag = (2*sin(a)*sin(a) + 0.087)* 0.5 *S * v*v;
        float lift = sin(2*a) * 0.5*S *v *v;

        //Drag force
        airdynamic_force = - local_velocity.normalized() * drag;

        //Lift vector
        Vector3f lift3f(-local_velocity.z(),0,local_velocity.x());
        lift3f = lift3f.normalized() * lift;
        airdynamic_force = airdynamic_force + lift3f;

        if (count % 10 == 1) {
//            ROS_INFO("local vel %f %f %f drag %5f lift %5f %d",local_velocity.x(),local_velocity.y(),local_velocity.z(),drag,lift,count);
//            ROS_INFO("Air Force %f %f %f",airdynamic_force.x(),airdynamic_force.y(),airdynamic_force.z());
        }
    }
//    return Vector3f(0,0,0);
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
    float damping_x = - airdynamic_damping_rate_x * angular_velocity.x() * angular_velocity.x();
    if (angular_velocity.x() < 0)
        damping_x = - damping_x;
    float damping_y = - airdynamic_damping_rate_y * angular_velocity.y() * angular_velocity.y();
    if (angular_velocity.y() < 0)
        damping_y = - damping_y;
    float damping_z = - airdynamic_damping_rate_z * angular_velocity.z() * angular_velocity.z();
    if (angular_velocity.z() < 0)
        damping_z = - damping_z;
    return Eigen::Vector3f(damping_x,damping_y,damping_z);
}
Eigen::Vector3f simulator::CalcTorque() {
    float LeftWing = actuators.axes[0];
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
//    Vector3f air_dynamic_torque = airdyamic_center.cross(air_dynamic_force()) + AirdynamicDamping();
    Vector3f air_dynamic_torque = AirdynamicDamping();
    return M + air_dynamic_torque;
}

int main(int argc,char ** argv)
{
    ros::init(argc, argv, "simulator");
    ros::NodeHandle nh;
    simulator sim(nh);
    ROS_INFO("SIMULATOR READY!!!");
    ros::spin();
}
