#include <ros/ros.h>
#include <kobuki_msgs/BumperEvent.h>
#include "iostream"
#include <std_msgs/Bool.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
//a boolean variable used to check if the bumper is activated.
bool check_bumper=true;
//Creating a struct called position. It is used to always check
//robots positioning.
struct position{
  float x_cor;
  float y_cor;
  float w_ori;
  float z_ori;
} current_position;
//the distance we want the robot to go backwards. Here 15 centimeters
const float distance_backwards=0.20;
//Function declaration. It makes the robot drive with a certain speed.
void driver(float speed);
//Function declaration for the bumper callback
void bumperCallback(kobuki_msgs::BumperEvent bump_msg);
//Function declaration for the odom callback
void odomCallback(nav_msgs::Odometry odom_msg);

void start_kiss();

void Navi_goals_CB(std_msgs::Bool msg){
    ros::NodeHandle nh;
    ros::Publisher Complete_pub = nh.advertise<std_msgs::Bool>("/kiss_me",1);

    std_msgs::Bool pub_msg;
    pub_msg.data = true;

    start_kiss();
    Complete_pub.publish(pub_msg);

}

int main(int argc, char *argv[]) {
  ros::init(argc,  argv, "kiss_me");
  //creating the different publishers and subscribers.
  ros::NodeHandle nh;
  //a publisher for the cmd_vel_mux/input/navi which controlls the speed
  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
  //Subscriber for the bumper
  ros::Subscriber sub=nh.subscribe("/mobile_base/events/bumper", 1000, bumperCallback);
  //Subscriber for the odometry
  ros::Subscriber o_sub= nh.subscribe("/odom", 1000, odomCallback);

  ros::Subscriber navi_sub = nh.subscribe("/simple_navigation_goals", 1, Navi_goals_CB);
  ros::Publisher Complete_pub = nh.advertise<std_msgs::Bool>("/kiss_me",1);
  //this while loop runs until the boolean variable changes to false
  //(see bumperCallback function)
  //The program does not run any further until check_bumper changes to false.
  ROS_INFO("Ready to kiss!");
  ros::spin();
  return 0;
}

void start_kiss(){
  check_bumper = true;

  while (check_bumper) {
    //a call for the driver function with the speed 0.1
    driver(0.1);
    //I think it makes the program spin(run again) one more time
    ros::spinOnce();
  }
  //we are making a constant called wall_position from the struct
  //position, which is declared above. This constant is set equal to
  //current_position since the robot is now positioned at the rack.
  position rack_position=current_position;
  //we creating a variable called distance which is measuring the
  //distance the robot has moved bsackwards.
  float distance=0;
  //creating a while that runs until the robot has moved the specified
  //distance backwards.
  while(distance<(distance_backwards)){
    //we are calling the driver function and makes the robot move
    //backwards. (the minus)
    driver(-0.1);
    //we are constantly calculating the distance the robot has moved
    //backwards. It is done with pythagoras theorem.
    distance=sqrt(pow(rack_position.x_cor-current_position.x_cor,2)
                   +pow(rack_position.y_cor-current_position.y_cor,2));
    std::cout << distance << std::endl;
    ros::spinOnce();
  }
}

  //the odom callback sets the position taken from the Odometry
  //equal to the struct current_position
  void odomCallback(nav_msgs::Odometry odom_msg){
    current_position.x_cor=odom_msg.pose.pose.position.x;
    current_position.y_cor=odom_msg.pose.pose.position.y;
    current_position.w_ori=odom_msg.pose.pose.orientation.w;
    current_position.z_ori=odom_msg.pose.pose.orientation.z;
  }
  //the driver function with a publisher to cmd_vel_mux/input/navi
  //It takes the speed we want the robot to go as a parameter.
  void driver(float speed){
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    geometry_msgs::Twist message;
    message.linear.x = speed;
    ros::Rate loop_rate(10);
    vel_pub.publish(message);
    loop_rate.sleep();
}
//the bumper callback function. It checks if the bumper has been
//activated with an if statement. If it is activated is sets the
//boolean variable check_bumper=false.
void bumperCallback(kobuki_msgs::BumperEvent bump_msg){
    if(bump_msg.state==1){
      check_bumper=false;
    }
  }
