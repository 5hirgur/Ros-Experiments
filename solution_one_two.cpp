
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include"turtlesim/Spawn.h"
#include <sstream>

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
turtlesim::Pose turtlesim_pose;


const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;

const double PI = 3.14159265359;

void move(double speed, double dist, bool fwrd);
void rotate (double anglrSpeed, double angle, bool clkwise);
double degrees2radians(double deg);
void setDesiredOrientation (double radAngle);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void moveGoal(turtlesim::Pose  goalPose, double distTolerence);
//void spawnRandomTurt(ros::NodeHandle );
void sweep();
void spawnRandomTurtle(ros::NodeHandle);


int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "turtlesim_cleaner");
	ros::NodeHandle n;
	double speed, anglrSpeed, dist, angle;
	bool fwrd, clkwise;


	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);
	
  ros::Rate loop(0.5);

	turtlesim::Pose pose;
  std::cout<<"Input (x,y) coordinates to move to:\nx:";
  double inputX;
  std::cin>>inputX;
  std::cout<<"y:";
  double inputY;
  std::cin>>inputY;
  pose.x = inputX;
  pose.y = inputY;

  moveGoal(pose, 0.01);

  int ctrlvar;
  bool spawnRanTurt = true;

  while(spawnRanTurt){
  spawnRandomTurtle(n);
  std::cout<<"\n press 0 to stop spawning turtles\n";
  std::cin>>ctrlvar;
  if(ctrlvar == 0){
  spawnRanTurt = false;}
  }
	loop.sleep();
	ros::spin();

	return 0;
}

//THIS WONT WORK !
// void spawnRandomTurt(ros::NodeHandle nh){
//   ros::ServiceClient spawnClient = nh.serviceClient<turtlesim::Spawn>("spawn");
//   turtlesim::Spawn spawnTurt;
//   spawnTurt.request.x = (double)rand() / (double)RAND_MAX * 11;
//   spawnTurt.request.y = (double)rand() / (double)RAND_MAX * 11;
//   if (spawnClient.call(spawnTurt))
//     {
//         ROS_INFO("Spawned turtle '%s' at position (%0.2f, %0.2f)", spawnTurt.response.name.c_str(), spawnTurt.response.x, spawnTurt.response.y);
//     }
//     else
//     {
//         ROS_ERROR("Failed to spawn turtle");
//     }
// }

void spawnRandomTurtle(ros::NodeHandle n)
{
    ros::ServiceClient spawnClient = n.serviceClient<turtlesim::Spawn>("/spawn");

    // create a "Spawn" service request message
    
    turtlesim::Spawn spawnReq;
    spawnReq.request.x = (float)rand() / RAND_MAX * 10.0; 
    spawnReq.request.y = (float)rand() / RAND_MAX * 10.0;
    spawnReq.request.theta = (float)rand() / RAND_MAX * 2.0 * M_PI; )
    spawnReq.request.name = "random_turtle";

    // call the "spawn" service and wait for a response
    if (spawnClient.call(spawnReq))
    {
        ROS_INFO("Turtle spawned at (%f, %f) with heading %f", spawnReq.request.x, spawnReq.request.y, spawnReq.request.theta);
          double inputX;
          double inputY;
          
          turtlesim::Pose pose;
          std::cout<<"Input (x,y) coordinates to move to:\nx:";
          std::cin>>inputX;
          std::cout<<"y:";
          std::cin>>inputY;
          pose.x = inputX;
          pose.y = inputY;     
          moveGoal(pose, 0.01);                 

  }
    else
    {
        ROS_ERROR("Failed to spawn turtle");
        
    }
}

void move(double speed, double distance, bool fwrd){
	geometry_msgs::Twist vel_msg;
	
	if (fwrd)
		vel_msg.linear.x =abs(speed);
	else
		vel_msg.linear.x =-abs(speed);
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;
	vel_msg.angular.z =0;

	double t0 = ros::Time::now().toSec();
	double current_distance = 0.0;
	ros::Rate loop_rate(100);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_publisher.publish(vel_msg);

}


void rotate (double anglrSpeed, double angle, bool clkwise){

	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x =0;
	vel_msg.linear.y =0;
	vel_msg.linear.z =0;
	vel_msg.angular.x = 0;
	vel_msg.angular.y = 0;

	if (clkwise)
		vel_msg.angular.z =-abs(anglrSpeed);
	else
		vel_msg.angular.z =abs(anglrSpeed);

	double current_angle = 0.0;
	double t0 = ros::Time::now().toSec();
	ros::Rate loop_rate(10);
	do{
		velocity_publisher.publish(vel_msg);
		double t1 = ros::Time::now().toSec();
		current_angle = anglrSpeed * (t1-t0);
		ros::spinOnce();
		loop_rate.sleep();
	}while(current_angle<angle);

	vel_msg.angular.z =0;
	velocity_publisher.publish(vel_msg);

}

double degrees2radians(double deg){
	return deg *PI /180.0;
}


void setDesiredOrientation (double radAngle){
	double relativeRadAngle = radAngle - turtlesim_pose.theta;
	bool clkwise = ((relativeRadAngle<0)?true:false);
	rotate (degrees2radians(10), abs(relativeRadAngle), clkwise);

}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}

double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

void moveGoal(turtlesim::Pose  goalPose, double distTolerence){

	geometry_msgs::Twist vel_msg;

	ros::Rate loop_rate(100);
	double E = 0.0;
	do{

		double Kp=1.0;
		double Ki=0.02;

		double e = getDistance(turtlesim_pose.x, turtlesim_pose.y, goalPose.x, goalPose.y);
		double E = E+e;
		vel_msg.linear.x = (Kp*e);
		vel_msg.linear.y =0;
		vel_msg.linear.z =0;
		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z =4*(atan2(goalPose.y-turtlesim_pose.y, goalPose.x-turtlesim_pose.x)-turtlesim_pose.theta);

		velocity_publisher.publish(vel_msg);

		ros::spinOnce();
		loop_rate.sleep();

	}while(getDistance(turtlesim_pose.x, turtlesim_pose.y, goalPose.x, goalPose.y)>distTolerence);
	vel_msg.linear.x =0;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}



void sweep(){

	ros::Rate loop(0.5);
	turtlesim::Pose pose;
	pose.x=1;
	pose.y=1;
	pose.theta=0;
	moveGoal(pose, 10.0);
	loop.sleep();
	setDesiredOrientation(0);
	loop.sleep();

	move(2.0, 9.0, true);
	loop.sleep();
	rotate(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	move(2.0, 9.0, true);


	rotate(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	move(2.0, 1.0, true);
	rotate(degrees2radians(10), degrees2radians(90), false);
	loop.sleep();
	move(2.0, 9.0, true);

	rotate(degrees2radians(30), degrees2radians(90), true);
	loop.sleep();
	move(2.0, 1.0, true);
	rotate(degrees2radians(30), degrees2radians(90), true);
	loop.sleep();
	move(2.0, 9.0, true);


	double distance = getDistance(turtlesim_pose.x, turtlesim_pose.y, x_max, y_max);

}





