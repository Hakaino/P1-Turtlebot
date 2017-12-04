#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <iostream>
#include <kobuki_msgs/Sound.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void initialize_shelves();

struct Coordinate {
    double x,y, rotW,rotZ;
};
#define AMOUNT_OF_SHELVES 8
#define ROTATION_UP (3.14)
#define ROTATION_DOWN ((3.14/4)*3)
Coordinate shelves[AMOUNT_OF_SHELVES];

using namespace std;


void prompt_for_input (int* Amount_of_entries) {


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>(
                                    "/visualization_marker_array", 1);
    ros::Publisher sound_pub = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);

    initialize_shelves();
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    int Entries= 0;
    int input_list[100];

    bool run = true;
    int input_shelf_number;


    while(run) {

        cout << "Everything below 1 and above the amount of shelves will activate the queue \n what shelf do you want to go to: \n";
        cin >> input_shelf_number;
            input_shelf_number--;

        if(input_shelf_number < 0 || input_shelf_number+1 > AMOUNT_OF_SHELVES) {
            run = false;
        } else {

            input_list[Entries] = input_shelf_number;
            Entries++;
        }
    }
    int failAmount = 0;
    kobuki_msgs::Sound Soundmsgs;
    Soundmsgs.value = 1;
    for(int i = 0; i < Entries; i++) {
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = shelves[input_list[i]].x;
        goal.target_pose.pose.position.y = shelves[input_list[i]].y;
        goal.target_pose.pose.orientation.w = shelves[input_list[i]].rotW;
        goal.target_pose.pose.orientation.z = shelves[input_list[i]].rotZ;

        ROS_INFO("Sending goal");
      //cout << "Sending goal! - shelf number: " << input_list[i]+1;
        ac.sendGoal(goal);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Sucess!");
          //  cout <<"Sucess! - shelf number: " << input_list[i]+1;
            Soundmsgs.value = 2;
            sound_pub.publish(Soundmsgs);
            }
        else{
            ROS_INFO("FAIL!");
          //  cout << "FAIL! - shelf number: " << input_list[i]+1;
            Soundmsgs.value = 1;
            sound_pub.publish(Soundmsgs);
            failAmount++;
            }
            ros::spinOnce();
    }

    Soundmsgs.value = 0;
    cout<< "\nDone with that wetn through: " << Entries << " positions, with " << failAmount << " fails \n";
    sound_pub.publish(Soundmsgs);
    return 0;
}

void _add_wp_markers() {
    /*
        ros::NodeHandle n;
        ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>(
                                        "/visualization_marker_array", 1);
        visualization_msgs::Marker marker[AMOUNT_OF_SHELVES];
        visualization_msgs::MarkerArray marker_array;

        for(int i = 0; i<AMOUNT_OF_SHELVES; i++) {
            marker[i].header.stamp = ros::Time::now();
            marker[i].ns = "navi";
            marker[i].type = visualization_msgs::Marker::ARROW;
            marker[i].action = visualization_msgs::Marker::ADD;
            marker[i].scale.x = 0.3;
            marker[i].scale.y = 0.8;
            marker[i].scale.z = 0.8;
            marker[i].color.r = 1.0;
            marker[i].color.g = 1.0;
            marker[i].color.b = 0.0;
            marker[i].color.a = 0.5;
            marker[i].pose.orientation.x = shelves[i].x;
            marker[i].pose.orientation.y = shelves[i].y;
            marker[i].pose.orientation.z = 0;
            marker[i].pose.orientation.w = shelves[i].rot;
            marker[i].lifetime = ros::Duration();
            marker[i].pose.position.x = shelves[i].x;
            marker[i].pose.position.y = shelves[i].y;
            marker[i].header.frame_id = "map";
            marker[i].id = i;
            marker_array.markers.push_back(marker[i]);
        }
        marker_pub.publish(marker_array);
    */
}

void initialize_shelves() { // 3
    int i = 0;
    float add = 0.38;
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
    // _add_wp_markers();

}
/*
void initialize_shelves() { // 2
    int i = 0;
    float add = 0.35;
    shelves[i].x = 0.150995731354;
    shelves[i].y = 0.368634879589 - add;
    shelves[i++].rot = ROTATION_UP;
    shelves[i].x = 0.398185491562;
    shelves[i].y = 0.368634879589 - add;
    shelves[i++].rot = ROTATION_UP;


    shelves[i].x = 0.219274416566;
    shelves[i].y = -0.532961428165 + add;
    shelves[i++].rot = ROTATION_UP;
    shelves[i].x = 0.415572166443;
    shelves[i].y = -0.532961428165 + add;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.219274416566;
    shelves[i].y = -0.532961428165 - add;
    shelves[i++].rot = ROTATION_UP;
    shelves[i].x = 0.415572166443;
    shelves[i].y = -0.532961428165 - add;
    shelves[i++].rot = ROTATION_UP;


    shelves[i].x = 0.164899230003;
    shelves[i].y = -1.73312473297 + add;
    shelves[i++].rot = ROTATION_UP;
    shelves[i].x = 0.346633434296 ;
    shelves[i].y = -1.72527873516 + add;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.164899230003;
    shelves[i].y = -1.73312473297 - add;
    shelves[i++].rot = ROTATION_UP;
    shelves[i].x = 0.346633434296 ;
    shelves[i].y = -1.72527873516 - add;
    shelves[i++].rot = ROTATION_UP;


    shelves[i].x = 0.116049975157;
    shelves[i].y = -2.38054728508 + add;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.355969488621;
    shelves[i].y = -2.37979006767 + add;
    shelves[i++].rot = ROTATION_UP;

    _add_wp_markers();
}*/
/*
void initialize_shelves(){ // FIrst try
    int i = 0;
    shelves[i].x = 0.15369579196;
    shelves[i].y = 0.152251243591;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.518825113773;
    shelves[i].y =  0.138516560197;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.166818380356;
    shelves[i].y =  -0.290963470936;
    shelves[i++].rot = ROTATION_DOWN;

    shelves[i].x = 0.171512305737;
    shelves[i].y =  -0.760341644287;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x =  0.449660509825;
    shelves[i].y =  -0.766253173351;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.100087746978;
    shelves[i].y =  -1.4391143322;
    shelves[i++].rot = ROTATION_DOWN;

    shelves[i].x = 0.421770006418;
    shelves[i].y =  -1.45328354836;
    shelves[i++].rot = ROTATION_DOWN;

    shelves[i].x = 0.131632208824;
    shelves[i].y =  -1.98646187782;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x = 0.427246242762;
    shelves[i].y =  -2.00089168549;
    shelves[i++].rot = ROTATION_UP;

    shelves[i].x =  0.106520175934;
    shelves[i].y =  -2.0823366642;
    shelves[i++].rot = ROTATION_DOWN;

    shelves[i].x = 0.44566822052;
    shelves[i].y =  -2.10502386093;
    shelves[i++].rot = ROTATION_DOWN;


}*/
