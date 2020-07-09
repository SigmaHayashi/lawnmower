#include "ros/ros.h"
#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"
#include "can_msgs/Frame.h"

// LawnMowerに送るコマンド
int command_speed[2] = {0}; // left rightの順

// LawnMowerから送られてくる情報
int lawnmower_engine_speed = 0; // [rpm]
int lawnmower_speed[2] = {0}; // left rightの順 [rpm]


// 何速で回すかを記録しておく
void callbackCaommandToLawnmower(const lawnmower::command_to_lawnmower::ConstPtr& msg){
    ROS_INFO("Command to LawnMower : %d, %d", msg->speed_left, msg->speed_right);
    command_speed[0] = msg->speed_left;
    command_speed[1] = msg->speed_right;
    //ROS_INFO("Command speed : %d, %d", command_speed[0], command_speed[1]);
}


// 草刈り機からの状態出力を変換
void callbackSocketcanToTopic(const can_msgs::Frame::ConstPtr& msg){
    //ROS_INFO("SoketCAN to Topic : %d", msg->id);
    if(msg->id == 0x418){
        lawnmower_engine_speed = msg->data[0];
        lawnmower_speed[0] = (msg->data[4] << 8) | msg->data[3];
        lawnmower_speed[1] = (msg->data[2] << 8) | msg->data[1];
        switch(msg->data[7]){
            case 0:
            lawnmower_speed[0] *= 0;
            lawnmower_speed[1] *= 0;
            break;

            case 1:
            lawnmower_speed[0] *= 1;
            lawnmower_speed[1] *= 0;
            break;
            
            case 2:
            lawnmower_speed[0] *= -1;
            lawnmower_speed[1] *= 0;
            break;
            
            case 3:
            lawnmower_speed[0] *= 0;
            lawnmower_speed[1] *= -1;
            break;
            
            case 4:
            lawnmower_speed[0] *= 1;
            lawnmower_speed[1] *= -1;
            break;
            
            case 5:
            lawnmower_speed[0] *= -1;
            lawnmower_speed[1] *= -1;
            break;
            
            case 6:
            lawnmower_speed[0] *= 0;
            lawnmower_speed[1] *= 1;
            break;
            
            case 7:
            lawnmower_speed[0] *= 1;
            lawnmower_speed[1] *= 1;
            break;
            
            case 8:
            lawnmower_speed[0] *= -1;
            lawnmower_speed[1] *= 1;
            break;
        }
        //ROS_INFO("LawnMower speed : %d, %d", lawnmower_speed[0], lawnmower_speed[1]);
        
        //double lawnmower_speed_left_mps = lawnmower_speed[0] * 14.06 / (2 * M_PI) / 60 / 1000;
        //double lawnmower_speed_right_mps = lawnmower_speed[1] * 14.06 / (2 * M_PI) / 60 / 1000;
        //ROS_INFO("LawnMower speed : %lf, %lf", lawnmower_speed_left_mps, lawnmower_speed_right_mps);
    }
}


// 草刈り機に送るコマンドを生成
boost::array<uint8_t, 8> makeCommandData(){
    boost::array<uint8_t, 8> data = {0};

    // 速度指令

    // 左クローラー
    if(command_speed[0] >= -5 && command_speed[0] <= 5){
        data[2] = 8 - command_speed[0];
    }
    else{
        ROS_WARN("Left Speed Error : %d", command_speed[0]);
        data[2] = 8;
    }

    // 右クローラー
    if(command_speed[1] >= -5 && command_speed[1] <= 5){
        data[1] = 8 - command_speed[1];
    }
    else{
        ROS_WARN("Right Speed Error : %d", command_speed[1]);
        data[1] = 8;
    }

    // その場旋回（回転優位の前進はできない）
    if(command_speed[0] * command_speed[1] < 0){
        //int min_command_speed = std::min(abs(command_speed[0]), abs(command_speed[1]));
        int min_command_speed = (abs(command_speed[0]) + abs(command_speed[1])) / 2;
        if(min_command_speed % 2 == 0){
            min_command_speed--;
        }
        if(command_speed[0] > 0){
            command_speed[0] = min_command_speed;
            command_speed[1] = min_command_speed * -1;
        }
        else{
            command_speed[0] = min_command_speed * -1;
            command_speed[1] = min_command_speed;
        }
        ROS_INFO("command_speed : %d %d", command_speed[0], command_speed[1]);

        if(command_speed[0] == 1 && command_speed[1] == -1){
            data[2] = 7;
            data[1] = 9;
        }
        else if(command_speed[0] == 3 && command_speed[1] == -3){
            data[2] = 5;
            data[1] = 11;
        }
        else if(command_speed[0] == 5 && command_speed[1] == -5){
            data[2] = 3;
            data[1] = 13;
        }
        else if(command_speed[0] == -1 && command_speed[1] == 1){
            data[2] = 9;
            data[1] = 7;
        }
        else if(command_speed[0] == -3 && command_speed[1] == 3){
            data[2] = 11;
            data[1] = 5;
        }
        else if(command_speed[0] == -5 && command_speed[1] == 5){
            data[2] = 13;
            data[1] = 3;
        }
        else{
            ROS_WARN("Speed Error : %d %d", command_speed[0], command_speed[1]);
            data[2] = 8;
            data[1] = 8;
        }
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
