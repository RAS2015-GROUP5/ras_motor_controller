#include "ros/ros.h"
#include <ras_arduino_msgs/PWM.h>
#include <ras_arduino_msgs/Encoders.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

static double deltaEncoderl=0;
static double deltaEncoderr=0;
static double encoderl=0;
static double encoderr=0;
static double encoderlFirst=0;
static double encoderrFirst=0;
static double desired_wl=0;
static double desired_wr=0;
static double desiredVelocity=0;
static double desiredAnglarVelocity=0;
static double desiredpwmLeft=0;
static ras_arduino_msgs::PWM pwm;


void encoderCallback(const ras_arduino_msgs::Encoders::ConstPtr& msg)
{
    if (encoderl==0)
    {
        encoderlFirst=-(double)msg->encoder1;
        encoderrFirst=-(double)msg->encoder2;
    }
    encoderl=-(double)msg->encoder1;
    encoderr=-(double)msg->encoder2;
    deltaEncoderl=-(double)msg->delta_encoder1;
    deltaEncoderr=-(double)msg->delta_encoder2;
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
template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
int main(int argc, char **argv)
{

    pwm.PWM1=0;
    pwm.PWM2=0;
    double K_pl=1; //1.005
    double K_pr=1; //0.9515
    double K_Il=0.2;
    double K_Ir=0.2;
    double estimated_wl;
    double estimated_wr;
    double wheelRadius=0.05;
    double robotBase=0.21;
    double sumleft=0;
    double sumright=0;
//    double zLeftdot=0;
//    double zLeft=0;
//    double ahatLeft=0;
//    double zRightdot=0;
//    double zRight=0;
//    double ahatRight=0;
//    double k=1;

    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle n;

    ros::Publisher PWMSignal = n.advertise<ras_arduino_msgs::PWM>("/arduino/pwm/", 1);
    ros::Subscriber encodersSignal = n.subscribe("/arduino/encoders", 1, encoderCallback);
    ros::Subscriber twistSignal = n.subscribe("/motor_controller/twist", 1, twistCallback);

    ros::Rate loop_rate(10); //hz

    ros::Time begin=ros::Time::now();
    while (ros::ok())
    {
        ros::spinOnce();
        desired_wl=(desiredVelocity-((robotBase*desiredAnglarVelocity)/2))/wheelRadius;
        desired_wr=(desiredVelocity+((robotBase*desiredAnglarVelocity)/2))/wheelRadius;
        estimated_wl = (deltaEncoderl*M_PI*10)/(180);
        estimated_wr = (deltaEncoderr*M_PI*10)/(180);

        //ROS_INFO("desired: %f, estimated: %g,u: %i",desired_wr,estimated_wr,pwm.PWM2);
        sumleft+=((desired_wl)*0.1);
        sumright+=((desired_wr)*0.1);


        pwm.PWM2 = saturate(pwm.PWM2 +(int)(K_pl*(desired_wl-estimated_wl)+K_Il*(sumleft-(M_PI*((encoderl-encoderlFirst))/180))),-255,255);
        pwm.PWM1 = saturate(pwm.PWM1 -(int)(K_pr*(desired_wr-estimated_wr)+K_Ir*(sumright-(M_PI*((encoderr-encoderrFirst))/180))),-255,255);
//        if(abs(estimated_wl)<abs(0.01*desired_wl)){
//            zLeftdot=k*5*pwm.PWM2*sgn(pwm.PWM2)/255;
//            zLeft=zLeft+zLeftdot*0.1;
//            ahatLeft=zLeft-k*abs(estimated_wl);
//            pwm.PWM2 = saturate(pwm.PWM2 +(int)(ahatLeft*sgn(pwm.PWM2)),-255,255);
//        }
//        if(estimated_wr!=0){
//            zRightdot=-k*5*pwm.PWM1*sgn(estimated_wr)/255;
//            zRight=zRight+zRightdot*0.1;
//            ahatRight=zRight-k*abs(estimated_wr);
//            pwm.PWM1 = saturate(pwm.PWM1 -(int)(ahatRight*sgn(estimated_wr)),-255,255);}
        ROS_INFO("%i,%i,%g,%g,%g,%g och ahat %g",pwm.PWM1,pwm.PWM2,desired_wl, estimated_wl,desired_wr, estimated_wr,ahatRight);

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
