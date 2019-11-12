
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/tf.h"
#include "math.h"
#include <termios.h>
double glo_x;
double glo_y;
double goal_x;
double goal_y;
double goal_th;
double target_th;
double roll, pitch, yaw;

tf2::Quaternion quat;

void globalvelCallback(const geometry_msgs::Twist& global_vel)
{
  glo_x = global_vel.linear.x;
  glo_y = global_vel.linear.y;
}
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    tf::Quaternion q(
        msgAMCL->pose.pose.orientation.x,
        msgAMCL->pose.pose.orientation.y,
        msgAMCL->pose.pose.orientation.z,
        msgAMCL->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}
void algo(double x,double y, double th)
{
 if((x*cos(th)) + (y*sin(th))>0)
	{
	   goal_x = x*cos(th) + y*sin(th);
	}
 else
	{
	   goal_x = 0;
	}
 target_th = atan2(y,x);
 goal_th = (target_th - th);
}
void th(double th_in,double alpha)
{
 double th_out;
 th_out = th_in;
 if(th_out < -M_PI)
	{
	    th_out = 2*M_PI + th_out;
	}
 else if (th_out > M_PI)
	{
	    th_out = th_out - 2*M_PI;
	}
 goal_th = alpha * th_out;
}
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("global_vel", 1000, globalvelCallback);
  ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 1000, amclCallback);
  geometry_msgs::Twist msg;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Rate rate(2);

 while(ros::ok())
 {
  int c = getch();  
  algo(glo_x,glo_y,yaw);
  th(goal_th,1);
  ROS_INFO_STREAM("goal_x" << goal_x );
  ROS_INFO_STREAM("goal_th" << goal_th );
  ROS_INFO_STREAM("target_th" << target_th); 
  msg.linear.x = goal_x;
  msg.angular.z = goal_th;
  pub.publish(msg);

  if(c=='s')
	{
	    ROS_INFO_STREAM("stop key is pressed");
	    msg.linear.x = 0;
	    msg.angular.z = 0;
	    pub.publish(msg);
            ros::shutdown();
	}
  rate.sleep();
  ros::spinOnce();

 }
}
