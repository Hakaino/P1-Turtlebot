#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <iostream>
#include <kobuki_msgs/Sound.h>
#include <std_msgs/Bool.h>

//declarations of functions
void initialize_shelves();
void kiss_me_CB(std_msgs::Bool kiss_msg);
std::queue<int> prompt_for_input(int *amount_of_entries);
int handle_goalReached(bool IsSucess);

//A struct used to store data on both locations and rotations
struct Coordinate {
    double x,y, rotW,rotZ;
};

//definitions and global variables
#define AMOUNT_OF_SHELVES 8
Coordinate shelves[AMOUNT_OF_SHELVES];
int kiss_complete;

using namespace std;
int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;

    ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);

    ros::Subscriber kiss_sub = n.subscribe<std_msgs::Bool>("/kiss_me", 100, kiss_me_CB);
    ros::Publisher kiss_pub = n.advertise<std_msgs::Bool>("/simple_navigation_goals", 1);

    initialize_shelves();
    //tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    int Entries = 0; // an int holding the total amount of goals sent
    int failAmount = 0; //an int holding the amount of fails

    queue<int> input_queue = prompt_for_input(&Entries);

    //A for loop that iterates through the different shelfs that was input by the user
    while( !input_queue.empty() ){ //While the queue is not empty
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = shelves[input_queue.front()].x;
        goal.target_pose.pose.position.y = shelves[input_queue.front()].y;
        goal.target_pose.pose.orientation.w = shelves[input_queue.front()].rotW;
        goal.target_pose.pose.orientation.z = shelves[input_queue.front()].rotZ;
        input_queue.pop();

        ROS_INFO("Sending goal");

        ac.sendGoal(goal); //Send the current goal to the move_base
        ac.waitForResult(); //wait until the turtlebot return either sucess or fail


        failAmount += handle_goalReached(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

        ros::spinOnce();
    }

    kobuki_msgs::Sound Soundmsgs; // a type to publish to the /mobile_base/commands/sound topic
    Soundmsgs.value = 0;
    cout << "\nDone with that went through: " << Entries << " positions, with "
         << failAmount << " fails \n";
    sound_pub.publish(Soundmsgs);
    return 0;
}

//kiss complete. -1 for nothing recieved from kiss_me node, 1 kiss_complete, 0 for kiss_not complete
void kiss_me_CB(std_msgs::Bool kiss_msg){
int kiss_complete;
   if(kiss_msg.data == true){
      kiss_complete = 1;
    } else{
      kiss_complete = 0;
    }
}

std::queue<int> prompt_for_input(int* amount_of_entries){
  bool run = true;
  int input_shelf_number = -1;
  queue<int> input_queue;
  *amount_of_entries = 0;
  //The while loop that prompts for inputs from the user
  while(run) {
      cout << "Everything below 1 and above the amount of shelves will activate the queue \n"
              "what shelf do you want to go to? \n"
              "Entry #" << *amount_of_entries++ <<": " ;
      cin >> input_shelf_number;
      input_shelf_number--; //subtract one from the input number to match the index of array

      //if the input is below or above the number of shelves we stop the prompts
      if(input_shelf_number < 0 || input_shelf_number+1 > AMOUNT_OF_SHELVES) {
          run = false;
      } else
      {// Else we put the user input into an array called "input_list"
          input_queue.push(input_shelf_number);
      }
  }
  return input_queue;
}

//function to handle the actions taken when the turtlebot reaches its goal
//whether or not it was reached sucessfully or not. If it was sucessfull the
//function return 0 if not it return 1.
int handle_goalReached(bool IsSucess){
  ros::NodeHandle n;

  ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
  kobuki_msgs::Sound Soundmsgs; // a type to publish to the /mobile_base/commands/sound topic

  ros::Publisher kiss_pub = n.advertise<std_msgs::Bool>("/simple_navigation_goals", 1);
  std_msgs::Bool kiss_msg;
  kiss_msg.data = true;

  ros::Rate loop_rate(5);

  if(IsSucess){
        ROS_INFO("Sucess!");
        Soundmsgs.value = 2;
        kiss_pub.publish(kiss_msg);
        sound_pub.publish(Soundmsgs);
        while (kiss_complete == -1) { // Wait for the kiss to complete
            ROS_INFO("waiting!");
            ros::spinOnce();
        }
        kiss_complete = -1; //reset the kiss_complete
        return 0;
  }
  else{
        ROS_INFO("FAIL!");
        Soundmsgs.value = 1;
        sound_pub.publish(Soundmsgs);
        return 1;
       }
}

void initialize_shelves() { // initialize the shelfes from the map
    int i = 0;
    float add = 0.38; //distance from the wall the goal is set

    shelves[i].x = -0.0106398761272;
    shelves[i].y = 0.375935912132 - add;
    shelves[i].rotZ = 0.713531741909;
    shelves[i++].rotW = 0.700622903771;

    shelves[i].x = 0.990336298943;
    shelves[i].y = 0.381471037865 - add;
    shelves[i].rotZ = 0.713531741909;
    shelves[i++].rotW = 0.700622903771;


    shelves[i].x = 0.00168535113335;
    shelves[i].y = -2.46735405922 + add;
    shelves[i].rotZ = -0.696203777866;
    shelves[i++].rotW = 0.717844202933;
    shelves[i].x = 0.938663363457;
    shelves[i].y = -2.4582092762 + add;
    shelves[i].rotZ = -0.711401400369;
    shelves[i++].rotW = 0.702785918721;

    shelves[i].x = -0.428755164146 + add;
    shelves[i].y = -1.11033499241;
    shelves[i].rotZ = 0.99998856758;
    shelves[i++].rotW = -0.00478170558912;
    shelves[i].x = 0.352906405926 - add;
    shelves[i].y = -1.06807863712;
    shelves[i].rotZ = 0.013401124585;
    shelves[i++].rotW = 0.999910200898;

    shelves[i].x = 0.449260354042 + add;
    shelves[i].y = -1.08679580688;
    shelves[i].rotZ = 0.999998656856;
    shelves[i++].rotW = -0.00163898955165;
    shelves[i].x = 1.34238398075 - add;
    shelves[i].y = -1.0309009552;
    shelves[i].rotZ = -0.0175797638454;
    shelves[i++].rotW = 0.999845464011;
}
