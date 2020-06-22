#include "ros/ros.h"
#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"
#include "can_msgs/Frame.h"

// LawnMowerに送るコマンド
int command_speed[2] = {0}; // left rightの順

// LawnMowerから送られてくる情報
int lawnmower_engine_speed = 0; // [rpm]
int lawnmower_speed[2] = {0}; // left rightの順 [rpm]


void callbackCaommandToLawnmower(const lawnmower::command_to_lawnmower::ConstPtr& msg){
    ROS_INFO("Command to LawnMower : %d, %d", msg->speed_left, msg->speed_right);
    command_speed[0] = msg->speed_left;
    command_speed[1] = msg->speed_right;
    //ROS_INFO("Command speed : %d, %d", command_speed[0], command_speed[1]);
}

void callbackSocketcanToTopic(const can_msgs::Frame::ConstPtr& msg){
    //ROS_INFO("SoketCAN to Topic : %d", msg->id);
    if(msg->id == 0x418){
        lawnmower_engine_speed = msg->data[0];
        lawnmower_speed[0] = (msg->data[4] << 8) | msg->data[3];
        lawnmower_speed[1] = (msg->data[2] << 8) | msg->data[1];
        //ROS_INFO("LawnMower speed : %d, %d", lawnmower_speed[0], lawnmower_speed[1]);
    }
}

boost::array<uint8_t, 8> makeCommandData(){
    boost::array<uint8_t, 8> data = {0};

    // 速度指令
    if(command_speed[0] == 2 && command_speed[1] == 2){ //2速前進
        data[2] = 6;
        data[1] = 6;
    }
    else if(command_speed[0] == 4 && command_speed[1] == 4){ //4速前進
        data[2] = 4;
        data[1] = 4;
    }
    else if(command_speed[0] == -2 && command_speed[1] == -2){ //2速後退
        data[2] = 10;
        data[1] = 10;
    }
    else if(command_speed[0] == -4 && command_speed[1] == -4){ //4速後退
        data[2] = 12;
        data[1] = 12;
    }
    else if(command_speed[0] == 4 && command_speed[1] == -4){ //その場右旋回
        data[2] = 4;
        data[1] = 12;
    }
    else if(command_speed[0] == -4 && command_speed[1] == 4){ //その場左旋回
        data[2] = 12;
        data[1] = 4;
    }
    else{ // 停止
        data[2] = 8;
        data[1] = 8;
    }

    return data;
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

        can_msgs::Frame msg_topic_to_socketcan;
        msg_topic_to_socketcan.id = 0x410;
        msg_topic_to_socketcan.dlc = 8;
        //msg_topic_to_socketcan.data = msg_data;
        msg_topic_to_socketcan.data = makeCommandData();
        pub_topic_to_socketcan.publish(msg_topic_to_socketcan);

        lawnmower::command_from_lawnmower msg_command_from_lawnmower;
        msg_command_from_lawnmower.engine_speed = lawnmower_engine_speed;
        msg_command_from_lawnmower.speed_left = lawnmower_speed[0];
        msg_command_from_lawnmower.speed_right = lawnmower_speed[1];
        pub_command_from_lawnmower.publish(msg_command_from_lawnmower);

        ros::spinOnce();

        loop_rate.sleep();
    }
    
}
