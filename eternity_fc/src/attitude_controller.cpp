#include <attitude_controller.h>
#include <iostream>
#include <vector>

#define deltatime 0.010f

void attitude_controller::init(ros::NodeHandle & nh) {
	init_sub(nh);

	mixer_pub = nh.advertise<sensor_msgs::Joy>("mixer",10);

	//init default values
	angular_velocity_sp.throttle = 0;
	angular_velocity_sp.wx = 0;
	angular_velocity_sp.wy = 0;
	angular_velocity_sp.wz = 0;

	attitude_sp.w = 1;
	attitude_sp.x = 0;
	attitude_sp.y = 0;
	attitude_sp.z = 0;
	attitude_sp.head_speed = 0;

    fast_timer = nh.createTimer(ros::Rate(100),&attitude_controller::fast_update,this);
	slow_timer = nh.createTimer(ros::Rate(10),&attitude_controller::slow_update,this);
}

void attitude_controller::slow_update(const ros::TimerEvent &timerEvent) {
	update_params(nh);
}

void attitude_controller::fast_update(const ros::TimerEvent &timerEvent) {
	run_controller(timerEvent);
}
void attitude_controller::run_controller(const ros::TimerEvent &timerEvent) {
	switch (mode)
	{
		case controller_mode::manual: {
			Vector3f angular_vel_sp;
			angular_vel_sp.x() = angular_velocity_sp.wx;
			angular_vel_sp.y() = angular_velocity_sp.wy;
			angular_vel_sp.z() = angular_velocity_sp.wz;
			angular_velocity_controller(deltatime, angular_vel_sp);
			Thrust = angular_velocity_sp.throttle;
			break;
		}
		case controller_mode::attitude:
		{
			Quaternionf att_sp;
			att_sp.w() = attitude_sp.w;
			att_sp.x() = attitude_sp.x;
			att_sp.y() = attitude_sp.y;
			att_sp.z() = attitude_sp.z;
			so3_attitude_controller(deltatime,att_sp);
			head_velocity_control(deltatime,attitude_sp.head_speed);
			break;
		}
		default:
			Aileron = 0;
			Elevator = 0;
			Thrust = 0;
			Rudder = 0;
	}
//	ROS_INFO("here")
	sensor_msgs::Joy joy;
	std::vector<float> axes(16);
	joy.axes = axes;
	joy.axes[0] = Aileron;
	joy.axes[1] = Elevator;
	joy.axes[2] = Thrust;
	joy.axes[3] = Rudder;
	mixer_pub.publish(joy);
}
void attitude_controller::init_sub(ros::NodeHandle &nh) {
	odometry_sub = nh.subscribe("/dji_sdk/odometry",10,&attitude_controller::odometry_callback,this);
	angular_velocity_sp_sub = nh.subscribe("/state_machine/angular_velocity_sp",10,&attitude_controller::angular_velocity_callback,this);
	attitude_sp_sub = nh.subscribe("/state_machine/attitude_sp",10,&attitude_controller::attitude_sp_callback,this);
	mode_sub = nh.subscribe("/state_machine/fc_mode",10,&attitude_controller::mode_sub_callback,this);
	accel_sub = nh.subscribe("/dji_sdk/Acceleration",10,&attitude_controller::accel_sub_callback,this);
}
void attitude_controller::attitude_sp_callback(const eternity_fc::attitude_sp &sp) {
	this->attitude_sp = sp;
}
void attitude_controller::angular_velocity_callback(const eternity_fc::angular_velocity_sp &sp) {

	this->angular_velocity_sp = sp;
}

void attitude_controller::mode_sub_callback(const std_msgs::Int32 &mode) {
	this->mode = static_cast<controller_mode>(mode.data);
}

void attitude_controller::accel_sub_callback(const dji_sdk::Acceleration &acc) {
	this->local_accel.x() = acc.ax;
	this->local_accel.y() = acc.ay;
	this->local_accel.z() = acc.az;
}

void attitude_controller::odometry_callback(const nav_msgs::Odometry &odometry) {
	this->odometry = odometry;
	//TODO:Remember this is body frame
	this->angular_velocity.x() = odometry.twist.twist.angular.x;
	this->angular_velocity.y() = odometry.twist.twist.angular.y;
	this->angular_velocity.z() = odometry.twist.twist.angular.z;

	this->attitude.w() = odometry.pose.pose.orientation.w;
	this->attitude.x() = odometry.pose.pose.orientation.x;
	this->attitude.y() = odometry.pose.pose.orientation.y;
	this->attitude.z() = odometry.pose.pose.orientation.z;

	this->ground_velocity.x() = odometry.twist.twist.linear.x;
	this->ground_velocity.y() = odometry.twist.twist.linear.y;
	this->ground_velocity.z() = odometry.twist.twist.linear.z;

	this->local_velocity = this->attitude.inverse()._transformVector(this->ground_velocity);
}


void attitude_controller::update_params(ros::NodeHandle & nh) {
	//rad
	nh.param("max_angular_velocity_x",max_angular_velocity.x(),1.0f);
	//rad
	nh.param("max_angular_velocity_y",max_angular_velocity.y(),1.0f);
	//rad
	nh.param("max_angular_velocity_z",max_angular_velocity.z(),1.0f);

	//TODO:Calcaute them
	nh.param("angular_velocity_p_x",angular_velocity_p.x(),1.0f);
	nh.param("angular_velocity_p_y",angular_velocity_p.y(),1.0f);
	nh.param("angular_velocity_p_z",angular_velocity_p.z(),1.0f);

	nh.param("attitude_p_x",attitude_p.x(),1.0f);
	nh.param("attitude_p_y",attitude_p.y(),1.0f);
	nh.param("attitude_p_z",attitude_p.z(),1.0f);

	nh.param("attitude_i_x",attitude_i.x(),0.1f);
	nh.param("attitude_i_y",attitude_i.y(),0.1f);
	nh.param("attitude_i_z",attitude_i.z(),0.1f);


	nh.param("attitude_d_x",attitude_d.x(),0.05f);
	nh.param("attitude_d_y",attitude_d.y(),0.05f);
	nh.param("attitude_d_z",attitude_d.z(),0.05f);


	nh.param("head_velocity_i",head_velocity_i,0.1f);
	nh.param("head_velocity_p",head_velocity_p,1.0f);
	nh.param("head_velocity_d",head_velocity_d,0.05f);

	nh.param("thrust_weight_ratio",thrust_weight_ratio,2.0f);
}

void attitude_controller::angle_axis_from_quat(Quaternionf q0, Quaternionf q_sp, float & angle, Vector3f & axis)
{
	Quaternionf relative = q0.inverse()*q_sp;
	static int count = 0;
	if (count++ % 10 == 0)
	{
		//UE_LOG(LogTemp, Log, TEXT("quarel %f x %f y %f z %f"), relative.w(), relative.x(), relative.y(), relative.z());
	}
	axis = Vector3f(relative.x(), relative.y(), relative.z());
	float axis_len = axis.norm();
	angle = 2 * atan2(axis_len, relative.w());
	if (angle > M_PI)
	{
		angle -= 2 * M_PI;
		axis = -axis;
	}
	else if (angle < -M_PI)
	{
		angle += 2 * M_PI;
		axis = -axis;
	}
	if (abs(relative.w()) < 0.999)
		axis.normalize();
}


void attitude_controller::angular_velocity_controller(float DeltaTime, Vector3f angular_vel_sp)
{
	static int count = 0;
	Vector3f err = angular_vel_sp - angular_velocity;
	Vector3f tmp = err;
	//filter

//	err = (err + angular_vel_err_last) / 2;
//	angular_vel_err_last = tmp;
//	ROS_INFO("ERR: %f %f %f",err.x(),err.y(),err.z());

	Aileron = err.x() * angular_velocity_p.x();// +err_d.x() * AngularVelocity_D;
	Elevator = err.y() * angular_velocity_p.y();// +err_d.y() * AngularVelocity_D;
	Rudder = err.z() * angular_velocity_p.z();// +err_d.z() * AngularVelocity_D;
}

Vector3f product(Vector3f a, Vector3f b)
{
	return Vector3f(a.x()*b.x(),a.y()*b.y(),a.z()*b.z());
}

void attitude_controller::so3_attitude_controller(float DeltaTime, Quaternionf attitude_sp)
{
	float angle = 0;
	Vector3f axis;
	angle_axis_from_quat(attitude, attitude_sp, angle, axis);

	//TODO:NEED I?
	Vector3f angular_vel_sp = angle * product(axis, attitude_p) -  product(attitude_d,angular_velocity);
	angular_velocity_controller(DeltaTime, angular_vel_sp);

	static int count = 0;
	if (count++ % 10 == 0)
	{
//		ROS_INFO( "angle %f axis %f %f %f", angle * 180 / M_PI, axis.x(), axis.y(), axis.z());
//		ROS_INFO("angular vel sp %f %f %f",angular_vel_sp.x(),angular_vel_sp.y(),angular_vel_sp.z());
	}
}

void attitude_controller::head_velocity_control(float DeltaTime, float head_velocity_sp)
{

	//TODO:Solve problem for not hovering
	float head_velocity = local_velocity.x();
	float err = head_velocity_sp - head_velocity;
	float err_d = local_accel.z();

	intt_head_velocity_err = intt_head_velocity_err + err * DeltaTime;
	if (intt_head_velocity_err / head_velocity_i > 5)
	{
		intt_head_velocity_err = 5 * head_velocity_i;
	}
	if (intt_head_velocity_err / head_velocity_i < -5)
	{
		intt_head_velocity_err = -5 * head_velocity_i;
	}


	//calc the vertical acc setpoint
	float acc_sp = err * head_velocity_p + err_d * head_velocity_d + intt_head_velocity_err * head_velocity_i;
	//inverse vertical acc sp to throttle
	float acc_sp_with_gra = acc_sp + 9.81;
	float throttle_acc = acc_sp_with_gra;

	Thrust = throttle_acc / thrust_weight_ratio /9.81;
	//Thrust from 0->1
}


int main(int argc,char ** argv)
{
    ros::init(argc,argv,"attitude_controller");
    ros::NodeHandle nh("attitude_controller");
    attitude_controller attitude_controller1(nh);
    ROS_INFO("Attitude Controller READY");

    ros::spin();
    return 0;
}
