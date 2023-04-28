#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <sstream>
#include <iostream>
#include <math.h>

using namespace std;

const double PI = 3.141592653589793238463;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_circle");
  ros::NodeHandle nh;

  ros::Publisher velocity_publisher = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
  ros::Publisher real_pose_publisher = nh.advertise<turtlesim::Pose>("/rt_real_pose", 1000);
  ros::Publisher noisy_pose_publisher = nh.advertise<turtlesim::Pose>("/rt_noisy_pose", 1000);

  ros::Rate loop_rate(10);

  double speed, radius;
  cout << "Enter speed: ";
  cin >> speed;
  cout << "Enter radius: ";
  cin >> radius;

  while (ros::ok())
  {
    turtlesim::Pose turtlesim_pose;
    turtlesim_pose.x = 5.5;
    turtlesim_pose.y = 5.5;
    turtlesim_pose.theta = 0;

    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = speed;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;
    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = speed / radius;

    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;

    while (current_angle < 2 * PI)
    {
      velocity_publisher.publish(vel_msg);

      double t1 = ros::Time::now().toSec();
      current_angle = speed / radius * (t1 - t0);
      turtlesim_pose.x = 5.5 + radius * cos(current_angle);
      turtlesim_pose.y = 5.5 + radius * sin(current_angle);
      turtlesim_pose.theta = current_angle;

      real_pose_publisher.publish(turtlesim_pose);

      // add random Gaussian noise to the pose
      turtlesim::Pose noisy_pose = turtlesim_pose;
      noisy_pose.x += 10 * ((double)rand() / RAND_MAX - 0.5);
      noisy_pose.y += 10 * ((double)rand() / RAND_MAX - 0.5);
      noisy_pose_publisher.publish(noisy_pose);

      ros::spinOnce();
      loop_rate.sleep();
    }

    // stop the turtle after completing the circle
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
