#include "ros/ros.h"
#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"
#include "can_msgs/Frame.h"

// LawnMowerに送るコマンド
int command_speed[2] = {0}; // left rightの順

// LawnMowerから送られてくる情報
int lawnmower_engine_speed = 0; // [rot/min]
int lawnmower_speed[2] = {0}; // left rightの順 [rot/min]

// 正回転・逆回転を切り替えるときに使うやつ
int rot_direction_balancer[2] = {0};
const int rot_direction_balancer_max = 2;

// モーターの回転方向を正しく制御できていないときに使う
int crawler_control_error_count = {0};
const int crawler_control_error_max = 20;
int crawler_control_error_fix_count = 0;
const int crawler_control_error_fix_max = 2;


// 何速で回すかを記録しておく
void callbackCaommandToLawnmower(const lawnmower::command_to_lawnmower::ConstPtr& msg){
    ROS_INFO("Command to LawnMower : %d, %d", msg->speed_left, msg->speed_right);

    //command_speed[0] = msg->speed_left;
    //command_speed[1] = msg->speed_right;
    
    // 正回転・逆回転を急に切り替えないようにする対応 (一度0を送る)
    // 左クローラー
    if(command_speed[0] * msg->speed_left < 0 || rot_direction_balancer[0] > 0){
        command_speed[0] = 0;
        rot_direction_balancer[0]++;

        if(rot_direction_balancer[0] >= rot_direction_balancer_max){
            rot_direction_balancer[0] = 0;
            //ROS_WARN("HERE %d", rot_direction_balancer[0]);
        }
    }
    else{
        command_speed[0] = msg->speed_left;
    }
        
    // 右クローラー
    if(command_speed[1] * msg->speed_right < 0 || rot_direction_balancer[1] > 0){
        command_speed[1] = 0;
        rot_direction_balancer[1]++;

        if(rot_direction_balancer[1] >= rot_direction_balancer_max){
            rot_direction_balancer[1] = 0;
        }
    }
    else{
        command_speed[1] = msg->speed_right;
    }

    // その場旋回についての対応（回転優位の前進・後退はできない）
    if(command_speed[0] * command_speed[1] < 0){
        //int min_command_speed = std::min(abs(command_speed[0]), abs(command_speed[1]));
        int ave_command_speed = (abs(command_speed[0]) + abs(command_speed[1])) / 2;
        /*
        // 2速4速のその場旋回はできない
        if(ave_command_speed % 2 == 0){
            ave_command_speed--;
        }
        */
        if(command_speed[0] > 0){
            command_speed[0] = ave_command_speed;
            command_speed[1] = ave_command_speed * -1;
        }
        else{
            command_speed[0] = ave_command_speed * -1;
            command_speed[1] = ave_command_speed;
        }
    }

    //ROS_INFO("Final Command speed : %d, %d", command_speed[0], command_speed[1]);
    //ROS_INFO("Balancer : %d, %d", rot_direction_balancer[0], rot_direction_balancer[1]);
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
            if(lawnmower_speed[0] != 0 || lawnmower_speed[1] != 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= 0;
            lawnmower_speed[1] *= 0;
            break;

            case 1:
            if(lawnmower_speed[0] == 0 || lawnmower_speed[1] != 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= 1;
            lawnmower_speed[1] *= 0;
            break;
            
            case 2:
            if(lawnmower_speed[0] == 0 || lawnmower_speed[1] != 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= -1;
            lawnmower_speed[1] *= 0;
            break;
            
            case 3:
            if(lawnmower_speed[0] != 0 || lawnmower_speed[1] == 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= 0;
            lawnmower_speed[1] *= -1;
            break;
            
            case 4:
            if(lawnmower_speed[0] == 0 || lawnmower_speed[1] == 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= 1;
            lawnmower_speed[1] *= -1;
            break;
            
            case 5:
            if(lawnmower_speed[0] == 0 || lawnmower_speed[1] == 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= -1;
            lawnmower_speed[1] *= -1;
            break;
            
            case 6:
            if(lawnmower_speed[0] != 0 || lawnmower_speed[1] == 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= 0;
            lawnmower_speed[1] *= 1;
            break;
            
            case 7:
            if(lawnmower_speed[0] == 0 || lawnmower_speed[1] == 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= 1;
            lawnmower_speed[1] *= 1;
            break;
            
            case 8:
            if(lawnmower_speed[0] == 0 || lawnmower_speed[1] == 0){
                crawler_control_error_count ++;
            }
            else{
                crawler_control_error_count = 0;
                crawler_control_error_fix_count = 0;
            }
            lawnmower_speed[0] *= -1;
            lawnmower_speed[1] *= 1;
            break;
        }
        //ROS_INFO("LawnMower speed : %d, %d", lawnmower_speed[0], lawnmower_speed[1]);
    }
}


// 草刈り機に送るコマンドを生成
boost::array<uint8_t, 8> makeCommandData(){
    boost::array<uint8_t, 8> data = {0};

    // 速度指令

    // クローラーが正しく制御できていない場合、一度停止
    if(crawler_control_error_count >= crawler_control_error_max){
        crawler_control_error_fix_count ++;
        if(crawler_control_error_fix_count >= crawler_control_error_fix_max){
            crawler_control_error_count = 0;
            crawler_control_error_fix_count = 0;
        }
        command_speed[0] = command_speed[1] = 0;
    }
    ROS_INFO("Crawler Control Error: %d", crawler_control_error_count);
    ROS_INFO("Final Command speed  : %d, %d", command_speed[0], command_speed[1]);

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

    //ROS_INFO("command_speed : %d, %d", command_speed[0], command_speed[1]);
    return data;
}


int main(int argc, char** argv){
    ros::init(argc, argv, "lawnmower_can_topic_adapter");

    ros::NodeHandle nh;

    ros::Subscriber sub_command_to_lawnmower = nh.subscribe("command_to_lawnmower", 10, callbackCaommandToLawnmower);
    ros::Publisher pub_topic_to_socketcan = nh.advertise<can_msgs::Frame>("sent_messages", 10);

    ros::Subscriber sub_socketcan_to_topic = nh.subscribe("received_messages", 10, callbackSocketcanToTopic);
    ros::Publisher pub_command_from_lawnmower = nh.advertise<lawnmower::command_from_lawnmower>("command_from_lawnmower", 10);

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
