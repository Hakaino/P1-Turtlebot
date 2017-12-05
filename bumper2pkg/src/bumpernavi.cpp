#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/rate.h>
#include <iostream>
#include <stdio.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/Sound.h>
#include <kobuki_msgs/Led.h>

#include <kobuki_msgs/MotorPower.h>
#include <kobuki_msgs/WheelDropEvent.h>


#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>



using namespace std;


uint8_t bumper;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//define the bumperCallback function
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
     cout<<" WE ARE IN CALLBACK!!!!!"<<endl;

    ros::NodeHandle n;
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
    ros::Publisher bumperPublisherSound = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
    ros::Publisher bumperPublisherLED = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
    ros::Subscriber bumperSubscriber = n.subscribe("/mobile_base/events/bumper", 1000, bumperCallback);

    ros::Rate r(10);

    //create a Twist message to publish the velocity

    bumper = msg->state;

    geometry_msgs::Twist velMessage;
    kobuki_msgs::Sound soundMessage;
    kobuki_msgs::Led lightMessage;

    if (bumper == 1)
    {
        cout<<"BUUUUUUUUUUUUUUUUUUUUUUUUUUUUMMMMMMMMMMMMMMMMPPPPPPPPPPPPPPPPPPPP!!!!!"<<endl;


    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.5;
    goal.target_pose.pose.orientation.w = -0.5;

    ROS_INFO("Sending goal");

    ac.sendGoal(goal);



    soundMessage.value = 4;
    bumperPublisherSound.publish (soundMessage);

    lightMessage.value = 3;
    bumperPublisherLED.publish (lightMessage);



    ac.waitForResult();

    soundMessage.value = 2;
    bumperPublisherSound.publish (soundMessage);

    lightMessage.value = 0;
    bumperPublisherLED.publish (lightMessage);

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
    }



        /*velMessage.linear.x = 0.0;
        velMessage.angular.z = 0.0;
        velocityPublisher.publish(velMessage);
        //create a Kobuki_msgs to publish light
        kobuki_msgs::Led lightMessage;
        lightMessage.value = 3;
        bumperPublisherLED.publish (lightMessage);
        //create a Kobuki_msgs to publish light

        kobuki_msgs::Sound soundMessage;
        soundMessage.value = 4;
        bumperPublisherSound.publish (soundMessage); */


        {
            r.sleep();


        }


    //publish the message
    velocityPublisher.publish(velMessage);

}

//ROS node entry point
int main(int argc, char **argv)
{

    ros::init(argc, argv, "bumpernavi");

    ros::NodeHandle n;
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
    ros::Publisher bumperPublisherSound = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
    ros::Publisher bumperPublisherLED = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
    ros::Subscriber bumperSubscriber = n.subscribe("/mobile_base/events/bumper", 1000, bumperCallback);


    ros::spin();

    return 0;
}



//
