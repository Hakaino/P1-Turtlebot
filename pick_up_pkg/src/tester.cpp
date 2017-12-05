#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc,  argv, "tester");
  system("rosrun pick_up_pkg kiss_me");
  return 0;
}
