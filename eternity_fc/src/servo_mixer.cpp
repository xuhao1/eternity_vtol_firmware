//
// Created by xuhao on 2016/4/5.
//

#include <servo_mixer.h>

inline float Power(float a,float b)
{
   return pow(a,b);
}

inline float Sqrt(float a)
{
   return sqrt(a);
}

void servo_mixer::init(ros::NodeHandle &nh) {
   actucator_control_pub = nh.advertise<sensor_msgs::Joy>("/dji_sdk/possess_control",10);

   nh.param("aileron_angle_ratio",aileron_angle_ratio,0.5f);
   nh.param("k_thrust_z_ratio",k_thrust_z_ratio,0.15f);

   before_mixer.axes = std::vector<float>(8);
   after_mixer.axes = std::vector<float>(8);
   for (int i = 0;i<8;i++)
   {
      before_mixer.axes[i] = 0;
      after_mixer.axes[i] = 0;
   }

   mode_sub = nh.subscribe("/state_machine/fc_mode",10,&servo_mixer::mode_sub_callback,this);
   before_mixer_sub = nh.subscribe("/attitude_controller/mixer",10,&servo_mixer::update_before_mixer,this);
   rc_possess_sub = nh.subscribe("/state_machine/angular_velocity_sp",10,&servo_mixer::update_rc_values,this);

   fast_timer = nh.createTimer(ros::Rate(50),&servo_mixer::fast_update,this);
//   slow_timer = nh.createTimer(ros::Rate(10),&servo_mixer::slow_update,this);
}

void servo_mixer::mode_sub_callback(const std_msgs::Int32 &mode) {
   this->mode = static_cast<controller_mode>(mode.data);

   //TODO::more complex
   if (this->mode > controller_mode::disarm && this->mode < controller_mode::mode_end)
   {
      this->arm = true;
   }
   else{
      this->arm = false;
   }
}
inline float rerange_servo(float v)
{
   if (v>1)
      return 1.001;
   if (v<-1)
      return -1.001;
}
void servo_mixer::update_rc_values(eternity_fc::angular_velocity_sp data) {
   this->rc_posses_data = data;
}
void servo_mixer::mixer()
{
   static int count = 0;
   count ++;
   Eigen::Vector3f u_xyz;
   float Mx = before_mixer.axes[0];
   float My = before_mixer.axes[1];
   float Mz = before_mixer.axes[3];
   float Thrust = before_mixer.axes[2];

   u_xyz << (-1.953624230872906e5*My*Mz - 1.0622537302872798e6*Mx*Thrust + 1.881710033408788e5*Mz*Thrust)/(3.1578475658491784e4*Power(Mz,2) - 2.898214227537045e6*Power(Thrust,2)),(-1.1088137526575953e5*Mx*Mz + 1.964187937464974e4*Power(Mz,2) - 1.8715899057442548e6*My*Thrust)/(3.1578475658491784e4*Power(Mz,2) - 2.898214227537045e6*Power(Thrust,2)),0.6958875712665018*Mz;

   float al = 0,ar =0,ul =0 ,ur =0;
   //u 2 actuator
   //mix
   al = (u_xyz.x() - u_xyz.y());
   ar = (-u_xyz.x() - u_xyz.y());
   if (Thrust < 0.1)
   {
      if (Thrust > 0) {
         ul = sqrt(Thrust);
         ur = sqrt(Thrust);
      }
      al = 0;
      ar = 0;
   }
   else {


      al = actuator_rerange(al, 50 * (1 - aileron_angle_ratio),50 * (1 + aileron_angle_ratio));
      al = actuator_rerange(al, 50 * (1 - aileron_angle_ratio),50 * (1 + aileron_angle_ratio));

      //ul from 0 to 1
      if (Thrust + u_xyz.z() * k_thrust_z_ratio > 0)
         ul = sqrt(Thrust + u_xyz.z() * k_thrust_z_ratio);

      //ul from 0 to 1
      if (Thrust - u_xyz.z() * k_thrust_z_ratio > 0)
         ur = sqrt(Thrust - u_xyz.z() * k_thrust_z_ratio);

      ul = actuator_rerange(ul,0,100,0,1);
      ur = actuator_rerange(ur,0,100,0,1);
   }

   after_mixer.axes[0] = al;
   after_mixer.axes[1] = ar;
   if (this->arm) {
      after_mixer.axes[2] = ul;
      after_mixer.axes[3] = ur;
      if (this->mode == controller_mode::debug_possess_control)
      {
//         ROS_INFO("try to possess control");
         after_mixer.axes[0] = actuator_rerange(rc_posses_data.wx);
         after_mixer.axes[1] = actuator_rerange(rc_posses_data.wy);
         after_mixer.axes[2] = actuator_rerange(rc_posses_data.throttle);
         after_mixer.axes[3] = actuator_rerange(rc_posses_data.wz);
      }
   }
   else
   {
      after_mixer.axes[0] = 0;
      after_mixer.axes[1] = 0;
      after_mixer.axes[2] = 0;
      after_mixer.axes[3] = 0;
   }

   //Trim

   for (int i = 0;i<8;i++)
   {
      if (isnan(after_mixer.axes[i])) {
         after_mixer.axes[i] = 0;
//         ROS_WARN("Axis:%d Nan",i);
      }
      else
         after_mixer.axes[i] = trim(after_mixer.axes[i]);
   }
   actucator_control_pub.publish(after_mixer);
}

float servo_mixer::actuator_rerange(float v, float lowwer, float upper,float lowwer_origin,float upper_origin) {
   float tmp = (v - lowwer_origin)/(upper_origin - lowwer_origin);
   return lowwer + tmp * (upper - lowwer);
}

void servo_mixer::fast_update(const ros::TimerEvent &event) {
   mixer();
}

void servo_mixer::slow_update(const ros::TimerEvent &event) {

}

void servo_mixer::update_before_mixer(sensor_msgs::Joy joy_data) {
   this->before_mixer = joy_data;
}

int main(int argc,char ** argv) {
   ros::init(argc, argv, "servo_mixer");
   ros::NodeHandle nh("servo_mixer");
   servo_mixer sm(nh);
   ROS_INFO("SERVO MIXER Controller READY");
   ros::spin();
}

