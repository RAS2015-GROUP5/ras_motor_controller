#include "ros/ros.h"
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
//#include "ros/time.h"
// Header for 'plant_msg.msg'
// #include "pid/plant_msg.h"

// // Header for controller_msg.msg
// #include "pid/controller_msg.h"
static double deltaEncoder1=0;
static double deltaEncoder2=0;
static double encoder1=0;
static double encoder2=0;
static double encoder1First=0;
static double encoder2First=0;
static double desired_w1=0;
static double desired_w2=0;
static double desiredVelocity=0;
static double desiredAnglarVelocity=0;
static double desiredpwmLeft=0;
static ras_arduino_msgs::PWM pwm;
// void PIDLeftSubCallback(const pid::controller_msg::ConstPtr& msg)
// {
//   pwm.PWM1=(int)msg->u;
// }
// void PIDRightSubCallback(const pid::controller_msg::ConstPtr& msg)
// {
//   pwm.PWM2=(int)msg->u;
// }

void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
  if (encoder1==0)
  {
   encoder1First=(double)msg->encoder1;
   encoder2First=(double)msg->encoder2;
  }
  encoder1=(double)msg->encoder1;
  encoder2=(double)msg->encoder2;
  deltaEncoder1=(double)msg->delta_encoder1;
  deltaEncoder2=(double)msg->delta_encoder2;
  ROS_INFO("I heard: [%i,%i,%i,%i]", msg->delta_encoder1, msg->delta_encoder2,msg->encoder1,msg->encoder2);
}
void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  desiredVelocity=msg->linear.x;
  desiredAnglarVelocity=msg->angular.z;
//  ROS_INFO("I heard: [%g,%g]", msg->linear.x,msg->angular.z);
}
int32_t saturate(int32_t val, int32_t min, int32_t max) {
    return std::min(std::max(val, min), max);
}

int main(int argc, char **argv)
{

    pwm.PWM1=0;
    pwm.PWM2=0;
    double K_pr=1.005; //1.005
    double K_pl=1.1; //0.9515
    double K_I=0.0;
    double estimated_w1;
    double estimated_w2;
    double wheelRadius=0.05;
    double robotBase=0.21;
    double sumleft=0;
    double sumright=0;
  // pid::plant_msg plantLeft;
  // pid::plant_msg plantRight;
  ros::init(argc, argv, "motor_controller");
  ros::NodeHandle n;
  // ros::Publisher PIDleft = n.advertise<pid::plant_msg>("/state/left", 1000);
  // ros::Publisher PIDright = n.advertise<pid::plant_msg>("/state/right", 1000);
  ros::Publisher PWMSignal = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm/", 1);
  ros::Subscriber encodersSignal = n.subscribe("/arduino/encoders", 1, encoderCallback);
  ros::Subscriber twistSignal = n.subscribe("/motor_controller/twist", 1, twistCallback);
  // ros::Subscriber PIDleftSub = n.subscribe("/control_effort/left", 1000, PIDLeftSubCallback);
  // ros::Subscriber PIDRightSub = n.subscribe("/control_effort/right", 1000, PIDRightSubCallback);
  ros::Rate loop_rate(10); //hz
  ros::Time begin=ros::Time::now();
  while (ros::ok())
  {
    ros::spinOnce();
    desired_w1=(desiredVelocity-((robotBase*desiredAnglarVelocity)/2))/wheelRadius;
    desired_w2=(desiredVelocity+((robotBase*desiredAnglarVelocity)/2))/wheelRadius;
    estimated_w1 = (deltaEncoder1*M_PI*10)/(180);
    estimated_w2 = (deltaEncoder2*M_PI*10)/(180);

    // plantLeft.x=estimated_w1;
    // plantLeft.setpoint=desired_w1;
    // plantLeft.t=(ros::Time::now()-begin).toSec();
    // PIDleft.publish(plantLeft);

    // plantRight.x=estimated_w2;
    // plantRight.setpoint=desired_w2;
    // plantRight.t=(ros::Time::now()-begin).toSec();
    // PIDright.publish(plantRight);

    //ROS_INFO("desired: %f, estimated: %g,u: %i",desired_w2,estimated_w2,pwm.PWM2);
    sumleft+=((desired_w1)*0.1);
    sumright+=((desired_w2)*0.1);

    pwm.PWM1 = saturate(pwm.PWM1 -(int)(K_pr*(desired_w1-estimated_w1)+K_I*(sumleft-(M_PI*((encoder1-encoder1First))/180))),-255,255);
    pwm.PWM2 = saturate(pwm.PWM2 +(int)(K_pl*(desired_w2-estimated_w2)+K_I*(sumright-(M_PI*((encoder2-encoder2First))/180))),-255,255);
    ROS_INFO("%i,%i,%g,%g,%g,%g",pwm.PWM1,pwm.PWM2,desired_w1, estimated_w1,desired_w2, estimated_w2);
/*
    if (desiredVelocity == 0.0 && desiredAnglarVelocity == 0.0){
        pwm.PWM1 =0;
        pwm.PWM2 =0;
    }*/


    PWMSignal.publish(pwm);



    loop_rate.sleep();
  }


  return 0;
}
