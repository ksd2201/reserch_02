
#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

int  joyPin1 = 1;                 // slider variable connecetd to analog pin 0
int  joyPin2 = 2;                 // slider variable connecetd to analog pin 1
int x_input = 0;                  // variable to read the value from the analog pin 0
int y_input = 0;                  // variable to read the value from the analog pin 1
double x_prime = 0;
double y_prime = 0;
double rad =0;
double x_val =0;
double y_val =0;
double goal_x =0;
double goal_y =0;
ros::NodeHandle  nh;
geometry_msgs::Twist twist;
ros::Publisher goal_vel("cmd_vel", &twist);

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
//remapping analog input(0~1023, x_val 0~0.22, y_val -2.8 ~ 2.8) to significant cmd_val
//make zero point
void joy()
{
  x_input = (double)map(x_input,0,1023,-512,511);
  y_input = (double)map(y_input,0,1023,511,-512);
 //x',y'
  rad = atan2(y_input, x_input);
  x_prime =  fabs(x_input) * (cos(rad));
  y_prime =  fabs(y_input) * (sin(rad));
 //x'', y''
  x_val = (x_prime / 512);
  y_val = (y_prime / 512);
  goal_x = x_val * 0.22;
  goal_y = y_val * 2.5;
  if(fabs(goal_x)<0.07)
  {
    goal_x=0; 
  }
  if(fabs(goal_y)<0.02)
  {
    goal_y=0; 
  }
}
void setup()
{
  nh.initNode();  
  nh.advertise(goal_vel);
  twist.linear.x = 0.0;
  twist.angular.z = 0.0;   
}

void loop()
{

 
  x_input = analogRead(joyPin1);
  y_input = analogRead(joyPin2);
  joy();
  Serial.println(goal_x);
  Serial.println(goal_y);
  twist.linear.x = goal_x;
  twist.angular.z = goal_y;

  goal_vel.publish(&twist);
  nh.spinOnce();
}
