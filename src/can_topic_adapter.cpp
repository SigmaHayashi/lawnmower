#include "ros/ros.h"
#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"
//#include "socketcan_bridge/socketcan_to_topic.h"
//#include "socketcan_bridge/topic_to_socketcan.h"
#include "can_msgs/Frame.h"

// left rightの順
int lawnmower_speed[2] = {0};
int command_speed[2] = {0};

void callbackCaommandToLawnmower(const lawnmower::command_to_lawnmower::ConstPtr& msg){
    ROS_INFO("Command to LawnMower : %d, %d", msg->speed_left, msg->speed_right);
    command_speed[0] = msg->speed_left;
    command_speed[1] = msg->speed_right;
    ROS_INFO("Command speed : %d, %d", command_speed[0], command_speed[1]);
}

void callbackSocketcanToTopic(const can_msgs::Frame::ConstPtr& msg){
    ROS_INFO("SoketCAN to Topic : %d", msg->id);
    if(msg->id == 0x418){
        lawnmower_speed[0] = (msg->data[4] << 8) | msg->data[3];
        lawnmower_speed[1] = (msg->data[2] << 8) | msg->data[1];
        ROS_INFO("LawnMower speed : %d, %d", lawnmower_speed[0], lawnmower_speed[1]);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "lawnmower_can_topic_adapter");

    ros::NodeHandle nh;

    ros::Subscriber sub_command_to_lawnmower = nh.subscribe("command_to_lawnmower", 1, callbackCaommandToLawnmower);
    ros::Publisher pub_topic_to_socketcan = nh.advertise<can_msgs::Frame>("sent_messages", 1);

    ros::Subscriber sub_socketcan_to_topic = nh.subscribe("received_messages", 1, callbackSocketcanToTopic);
    ros::Publisher pub_command_from_lawnmower = nh.advertise<lawnmower::command_from_lawnmower>("command_from_lawnmower", 1);

    ROS_INFO("Start CAN Topic Adapter");

    ros::Rate loop_rate(20);

    while(ros::ok()){
        /*
        lawnmower::command_from_lawnmower msg;
        msg.engine_speed = 0;
        msg.speed_left = 0;
        msg.speed_right = 0;

        pub_command_from_lawnmower.publish(msg);
        */

        boost::array<uint8_t, 8> msg_data;
        /*
        if(command_speed[0] > 0){
            msg_data[2] = 6;
        }
        else if(command_speed[0] < 0){
            msg_data[2] = 10;
        }
        else{
            msg_data[2] = 8;
        }
        
        if(command_speed[1] > 0){
            msg_data[1] = 6;
        }
        else if(command_speed[1] < 0){
            msg_data[1] = 10;
        }
        else{
            msg_data[1] = 8;
        }*/
        if(command_speed[0] == 1 && command_speed[1] == 1){
            msg_data[2] = 6;
            msg_data[1] = 6;
        }
        else if(command_speed[0] == 2 && command_speed[1] == 2){
            msg_data[2] = 4;
            msg_data[1] = 4;
        }
        else if(command_speed[0] == -1 && command_speed[1] == -1){
            msg_data[2] = 10;
            msg_data[1] = 10;
        }
        else if(command_speed[0] == -2 && command_speed[1] == -2){
            msg_data[2] = 12;
            msg_data[1] = 12;
        }
        else if(command_speed[0] == 2 && command_speed[1] == -2){
            msg_data[2] = 4;
            msg_data[1] = 12;
        }
        else if(command_speed[0] == -2 && command_speed[1] == 2){
            msg_data[2] = 12;
            msg_data[1] = 4;
        }
        else{
            msg_data[2] = 8;
            msg_data[1] = 8;
        }

        can_msgs::Frame msg_topic_to_socketcan;
        msg_topic_to_socketcan.id = 0x410;
        msg_topic_to_socketcan.dlc = 8;
        msg_topic_to_socketcan.data = msg_data;
        pub_topic_to_socketcan.publish(msg_topic_to_socketcan);

        lawnmower::command_from_lawnmower msg_command_from_lawnmower;
        msg_command_from_lawnmower.speed_left = lawnmower_speed[0];
        msg_command_from_lawnmower.speed_right = lawnmower_speed[1];
        pub_command_from_lawnmower.publish(msg_command_from_lawnmower);

        ros::spinOnce();

        loop_rate.sleep();
    }

    //ros::spin();
}
