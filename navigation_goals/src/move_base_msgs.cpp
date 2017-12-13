#include <ros/ros.h>
#include "math.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <list>
#include <iostream>
#include <kobuki_msgs/Sound.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/GetPlan.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

//declarations of functions
void initialize_shelves();
void kiss_me_CB(std_msgs::Bool kiss_msg);
std::list<int> prompt_for_input(int &amount_of_entries);
int handle_goalReached(bool IsSucess, ros::Publisher sound_pub, ros::Publisher kiss_pub, ros::NodeHandle n);
void SortList(std::list<int> &input_list);
bool Call_makePlan_service(ros::ServiceClient &MakePlanClient, nav_msgs::GetPlan &Service_msg);
float find_lenght_of_path(nav_msgs::GetPlan msgWithPlan);
int find_closest_goal(ros::ServiceClient &MakePlanClient,
                      std::list<int> sorted_inputList);
move_base_msgs::MoveBaseGoal getGoalFromShelf( int closest );

//A struct used to store data on both locations and rotations
struct Coordinate {
    double x,y, rotW,rotZ;
};

//definitions and global variables
#define AMOUNT_OF_SHELVES 8
Coordinate shelves[AMOUNT_OF_SHELVES];
int kiss_complete;


int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;


  geometry_msgs::PoseStamped NewTemp;
  

    ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
    ros::Subscriber kiss_sub = n.subscribe<std_msgs::Bool>("/kiss_me", 100, kiss_me_CB);
    ros::Publisher kiss_pub = n.advertise<std_msgs::Bool>("/simple_navigation_goals", 1);
    
    //A service client that can get a plan from the move_base/make_plan service
    ros::ServiceClient MakePlanClient = n.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan",true);

    //tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    //run the function to initialize the hardcoded shelf coordinates
    initialize_shelves();

    //waiting for the move_base server to come up
    while( !ac.waitForServer(ros::Duration(1.0)) ) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int Entries = 0; // an int holding the total amount of goals sent
    int failAmount = 0; //an int holding the amount of fails

    list<int> input_list = prompt_for_input(Entries); // get input from the user
    SortList(input_list); //sorts the list numerically
    list<int> FailedList;
    bool HasTriedFailedList = false; //A bool that makes sure that we only run the FailedList once
    //A for loop that iterates through the different shelfs that was input by the user
    while( !input_list.empty() ){ //While the queue is not empty

      //Find the closest shelf, and put that into a variable
      int closest = find_closest_goal(MakePlanClient , input_list);

      ROS_INFO("Sending goal, shelf # %d", closest+1);

      ac.sendGoal(getGoalFromShelf(closest)); //Send the current goal to the move_base
      ac.waitForResult(); //wait until the turtlebot return either sucess or fail

      bool GoalSuccess = ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
      //If we failed the goal, we want to add it to the failedlist, so we can retry it later
      if(handle_goalReached(GoalSuccess, sound_pub, kiss_pub,n) != 0){
        FailedList.push_front(closest);
      }
      input_list.remove(closest);// remove the shelf we just visited from the list
      
      //spin to update Everything
      ros::spinOnce();

      //an if statement that reattempts to reach any failed goals.
      if( !HasTriedFailedList && input_list.empty() && !FailedList.empty()){
        copy(FailedList.begin(), FailedList.end(), input_list.begin());
        SortList(input_list);
        HasTriedFailedList = true;
      }
    }

    kobuki_msgs::Sound Soundmsgs; // a type to publish to the /mobile_base/commands/sound topic
    Soundmsgs.value = 0;
    //give feedback to the user before closing the program
    cout << "\nDone with that went through: " << Entries << " positions, with "
         << FailedList.size() << " fails \n";
    sound_pub.publish(Soundmsgs); // play a sound before we quit
    return 0;
}

//kiss complete. -1 for nothing recieved from kiss_me node, 1 kiss_complete, 0 for kiss_not complete
void kiss_me_CB(std_msgs::Bool kiss_msg){ //sets other data
   if(kiss_msg.data == true){
      kiss_complete = 1;
    } else{
      kiss_complete = 0;
    }
}

std::list<int> prompt_for_input(int &amount_of_entries){
  bool run = true;
  int input_shelf_number = -1;
  list<int> input_queue;
  amount_of_entries = 1;
  //The while loop that prompts for inputs from the user
  while(run) {
      cout << "Everything below 1 and above the amount of shelves will activate the queue \n"
              "what shelf do you want to go to? \n"
              "Entry #" << amount_of_entries <<": ";
      cin >> input_shelf_number; //get data from user and put it into input_shelf_number

      input_shelf_number--; //subtract one from the input number to match the index of array

      //if the input is below or above the number of shelves we stop the prompts
      if(input_shelf_number < 0 || input_shelf_number+1 > AMOUNT_OF_SHELVES) {
          run = false; //will stop the while loop from looping again
      } else
      {// Else we put the user input into an array called "input_list"
      amount_of_entries += 1; // count up the amount of entries to keep track of how many shelves we visit
      input_queue.push_back(input_shelf_number);
      }
  }
  
  return input_queue;
}

void SortList(list<int> &input_list){
  std::vector<int> tmpVector;
  //fill in the vector for easier handling
  BOOST_FOREACH(int i, input_list){
    tmpVector.push_back(i);
  }
  //Bublesort the array
  //comparisons will be done n times
  for (int i = 0; i < tmpVector.size(); i++)
  {
      //compare elemet to the next element, and swap if condition is true
      for(int j = 0; j < tmpVector.size() - 1; j++)
      {
          if (tmpVector[j] > tmpVector[j+1])
          {
              int temp;
              temp = tmpVector[j];
              tmpVector[j] = tmpVector[j+1];
              tmpVector[j+1] = temp;
          }
      }
  }
  //put the array into the queue again
  copy(tmpVector.begin(),tmpVector.end(),input_list.begin());
}

//function to handle the actions taken when the turtlebot reaches its goal
//whether or not it was reached sucessfully or not. If it was sucessfull the
//function return 0 if not it return 1.
int handle_goalReached(bool IsSucess, ros::Publisher sound_pub, ros::Publisher kiss_pub, ros::NodeHandle n){
  kiss_complete = -1;//reset the kiss_complete
  kobuki_msgs::Sound Soundmsgs; // a type to publish to the /mobile_base/commands/sound topic

  std_msgs::Bool kiss_msg;
  ros::Rate r(10);
  if(IsSucess){
        ROS_INFO("Sucess!");
        Soundmsgs.value = 2;
        kiss_pub.publish(kiss_msg);
        sound_pub.publish(Soundmsgs);
        while (kiss_complete == -1) { // Wait for the kiss to complete
            ROS_INFO("waiting!");
            ros::spinOnce();
            r.sleep();
        }
        return 0;
  }
  else{
        ROS_INFO("FAIL!");
        Soundmsgs.value = 1;
        sound_pub.publish(Soundmsgs);
        return 1;
       }
}

//The function that calls the service and updates the path in the service_msg
bool Call_makePlan_service(ros::ServiceClient &MakePlanClient, nav_msgs::GetPlan &Service_msg){
    if(MakePlanClient.call(Service_msg)){
      return true; //return true if it was SUCCEEDED
    }else {
      ROS_ERROR("Failed to call make_plan service");
      return false; //False if the call to the client failed.
    }
}

int find_closest_goal(ros::ServiceClient &MakePlanClient,
                      std::list<int> sorted_inputList){
    int Closest_Shelf = 0; //the value that is eventually returned by the function
    int LastEntry = -1; //last entry set to -1 to ensure that it is not a shelf by default
    float shortestPath = 100.0; //variable to hold the current shortest known path
    BOOST_FOREACH(int i, sorted_inputList){ //A loop that iterates through the list
      if(i != LastEntry){
        //Create a new msgs of the type required for move_base/make_plan
        nav_msgs::GetPlan ServerMsg;
        //input the data into the msgs that should be sent to the service
        ServerMsg.request.goal.header.frame_id = "map";
        ServerMsg.request.goal.header.stamp = ros::Time::now();
        ServerMsg.request.goal.pose.position.x = shelves[i].x;
        ServerMsg.request.goal.pose.position.y = shelves[i].y;
        ServerMsg.request.goal.pose.orientation.w = shelves[i].rotW;
        ServerMsg.request.goal.pose.orientation.z = shelves[i].rotZ;

        //Make the plan
        Call_makePlan_service(MakePlanClient, ServerMsg);
        //find the lenght to the newly found plan.
        float tmpPathLenght = find_lenght_of_path(ServerMsg);

        if(tmpPathLenght < 0.001){ //if we get a zero lenght path we want to consider it the longest possible
          tmpPathLenght = 10.00;
        }
        ROS_INFO("Checking position %d with the lenght of %f", i+1, tmpPathLenght);

        //if the newly found path lenght is less than the previously shortestPath
        if(tmpPathLenght < shortestPath){
          shortestPath = tmpPathLenght; //update the shortest path
          Closest_Shelf = i; // Remember which shelf this path went to
        }
        LastEntry = i;
      }
    }
    return Closest_Shelf;
}

move_base_msgs::MoveBaseGoal getGoalFromShelf( int closest ){
  move_base_msgs::MoveBaseGoal goal; //create a variable to return
  //assign the values to the goal from the shelves array
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = shelves[closest].x;
  goal.target_pose.pose.position.y = shelves[closest].y;
  goal.target_pose.pose.orientation.w = shelves[closest].rotW;
  goal.target_pose.pose.orientation.z = shelves[closest].rotZ;
  return goal;
}

//Find the lenght of a given path. The path is within the nav_msgs::getPlan
float find_lenght_of_path(nav_msgs::GetPlan msgWithPlan){
  float Total_lenght = 0.0;
  bool firstTime = true;
  geometry_msgs::PoseStamped p1, p2;
  //BOOST_FOREACH iterates through the positions that make up the path.
  BOOST_FOREACH(geometry_msgs::PoseStamped &i, msgWithPlan.response.plan.poses){
    if(firstTime == true){ // setting the first element to p1 the first time around
      p1 = i;
      firstTime = false;
    }else{ //calculating the distance between two points on the path and accumilating it in Total_lenght
      p2 = i;
      Total_lenght += sqrt((p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x)
        + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y));
      p1 = i;
    }
  }
  return Total_lenght;
}

void initialize_shelves() { // initialize the shelves from the map
    int i = 0;
    float add = 0.38; //distance from the wall the goal is set

    shelves[i].x = -0.0106398761272;
    shelves[i].y = 0.375935912132 - add;
    shelves[i].rotZ = 0.713531741909;
    shelves[i++].rotW = 0.700622903771;
    shelves[i].x = 0.990336298943;
    shelves[i].y = (0.381471037865 - add) -0.1;
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
