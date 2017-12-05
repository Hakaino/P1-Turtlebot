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


using namespace std;


uint8_t bumper;

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
        velMessage.linear.x = 0.0;
        velMessage.angular.z = 0.0;
        velocityPublisher.publish(velMessage);
        //create a Kobuki_msgs to publish light
        kobuki_msgs::Led lightMessage;
        lightMessage.value = 3;
        bumperPublisherLED.publish (lightMessage);
        //create a Kobuki_msgs to publish light
        kobuki_msgs::Sound soundMessage;
        soundMessage.value = 4;
        bumperPublisherSound.publish (soundMessage);

        for (int t=0; t<50; t++)
        {
            r.sleep();
            if (t>25 && t<45)

                velMessage.linear.x = -0.1;
                velMessage.angular.z = 0.0;
                velocityPublisher.publish(velMessage);

                kobuki_msgs::Sound soundMessage;
                soundMessage.value = 2;
                bumperPublisherSound.publish (soundMessage);

                kobuki_msgs::Led lightMessage;
                lightMessage.value = 4;
                bumperPublisherLED.publish (lightMessage);

            if (t==50)

                velMessage.linear.x = 0.0;
                velMessage.angular.z = 0.0;
                velocityPublisher.publish(velMessage);


                soundMessage.value = 2;
                bumperPublisherSound.publish (soundMessage);


                lightMessage.value = 3;
                bumperPublisherLED.publish (lightMessage);
         {
            r.sleep();


        }

        }
    }

    //publish the message
    velocityPublisher.publish(velMessage);

}

//ROS node entry point
int main(int argc, char **argv)
{
    ros::init(argc, argv, "bumper123");

    ros::NodeHandle n;
    ros::Publisher velocityPublisher = n.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/navi", 1);
    ros::Publisher bumperPublisherSound = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 1);
    ros::Publisher bumperPublisherLED = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 1);
    ros::Subscriber bumperSubscriber = n.subscribe("/mobile_base/events/bumper", 1000, bumperCallback);

    ros::spin();

    return 0;
}



//
