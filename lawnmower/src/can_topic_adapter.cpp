/*
#include "ros/ros.h"
#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"
#include "can_msgs/Frame.h"
*/
#include "rclcpp/rclcpp.hpp"
#include "lawnmower_msgs/msg/command_to_lawnmower.hpp"
#include "lawnmower_msgs/msg/command_from_lawnmower.hpp"
#include "can_msgs/msg/frame.hpp"
#include "std_msgs/msg/empty.hpp"

rclcpp::Node::SharedPtr node = nullptr;

// LawnMowerに送るコマンド
int command_speed[2] = {0}; // left rightの順
bool command_grass = false;

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

std::shared_ptr<rclcpp::Publisher<can_msgs::msg::Frame>> pub_topic_to_socketcan;
std::shared_ptr<rclcpp::Publisher<lawnmower_msgs::msg::CommandFromLawnmower>> pub_command_from_lawnmower;

// 何速で回すかを記録しておく
//void callbackCaommandToLawnmower(const lawnmower::command_to_lawnmower::ConstPtr& msg){
void callbackCaommandToLawnmower(const lawnmower_msgs::msg::CommandToLawnmower::SharedPtr msg){
    //ROS_INFO("Command to LawnMower : %d, %d", msg->speed_left, msg->speed_right);
    RCLCPP_INFO(node->get_logger(), "Command to LawnMower : %d, %d", msg->speed_left, msg->speed_right);

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
//void callbackSocketcanToTopic(const can_msgs::Frame::ConstPtr& msg){
void callbackSocketcanToTopic(const can_msgs::msg::Frame::SharedPtr msg){
    //ROS_INFO("SoketCAN to Topic : %d", msg->id);
    //if(msg->id == 0x418){
    //RCLCPP_INFO(node->get_logger(), "CAN msg ID: %x", msg->id);
    if(msg->id == 0x320){
        lawnmower_engine_speed = msg->data[0];
        lawnmower_speed[0] = (msg->data[4] << 8) | msg->data[3];
        lawnmower_speed[1] = (msg->data[2] << 8) | msg->data[1];
        RCLCPP_INFO(node->get_logger(), "LawnMower speed (raw): %d, %d", lawnmower_speed[0], lawnmower_speed[1]);
        RCLCPP_INFO(node->get_logger(), "Rotation Direction: %d", msg->data[7]);
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
        //RCLCPP_INFO(node->get_logger(), "LawnMower speed : %d, %d", lawnmower_speed[0], lawnmower_speed[1]);
    }
}

void callbackGrassStart(const std_msgs::msg::Empty::SharedPtr msg){
//void callbackGrassStart(){
    RCLCPP_INFO(node->get_logger(), "Grass Start!!");
    command_grass = true;
}

void callbackGrassStop(const std_msgs::msg::Empty::SharedPtr msg){
//void callbackGrassStop(){
    RCLCPP_INFO(node->get_logger(), "Grass Stop!!");
    command_grass = false;
}


// 草刈り機に送るコマンドを生成
//boost::array<uint8_t, 8> makeCommandData(){
std::array<uint8_t, 8> makeCommandData(){
    //boost::array<uint8_t, 8> data = {0};
    std::array<uint8_t, 8> data = {0};

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
    //ROS_INFO("Crawler Control Error: %d", crawler_control_error_count);
    //ROS_INFO("Final Command speed  : %d, %d", command_speed[0], command_speed[1]);
    RCLCPP_INFO(node->get_logger(), "Crawler Control Error: %d", crawler_control_error_count);
    RCLCPP_INFO(node->get_logger(), "Final Command speed  : %d, %d", command_speed[0], command_speed[1]);

    // 左クローラー
    if(command_speed[0] >= -5 && command_speed[0] <= 5){
        data[2] = 8 - command_speed[0];
    }
    else{
        //ROS_WARN("Left Speed Error : %d", command_speed[0]);
        data[2] = 8;
    }

    // 右クローラー
    if(command_speed[1] >= -5 && command_speed[1] <= 5){
        data[1] = 8 - command_speed[1];
    }
    else{
        //ROS_WARN("Right Speed Error : %d", command_speed[1]);
        data[1] = 8;
    }

    // 刈り取り
    if(command_grass){
        data[3] = 1;
    }
    else{
        data[3] = 0;
    }

    //ROS_INFO("command_speed : %d, %d", command_speed[0], command_speed[1]);
    return data;
}

void timerCallback(){
//void timerCallback(std::shared_ptr<rclcpp::Publisher<can_msgs::msg::Frame>> pub_to_socketcan, std::shared_ptr<rclcpp::Publisher<lawnmower_msgs::msg::CommandFromLawnmower>> pub_from_lawnmower){
//void timerCallback(rclcpp::Publisher<can_msgs::msg::Frame> pub_to_socketcan, rclcpp::Publisher<lawnmower_msgs::msg::CommandFromLawnmower> pub_from_lawnmower){
    can_msgs::msg::Frame msg_topic_to_socketcan;
    //msg_topic_to_socketcan.id = 0x410;
    msg_topic_to_socketcan.id = 0x310;
    msg_topic_to_socketcan.dlc = 8;
    msg_topic_to_socketcan.data = makeCommandData();
    //pub_topic_to_socketcan.publish(msg_topic_to_socketcan);
    pub_topic_to_socketcan->publish(msg_topic_to_socketcan);
    //pub_to_socketcan->publish(msg_topic_to_socketcan);

    //lawnmower::command_from_lawnmower msg_command_from_lawnmower;
    lawnmower_msgs::msg::CommandFromLawnmower msg_command_from_lawnmower;
    msg_command_from_lawnmower.engine_speed = lawnmower_engine_speed;
    msg_command_from_lawnmower.speed_left = lawnmower_speed[0];
    msg_command_from_lawnmower.speed_right = lawnmower_speed[1];
    //pub_command_from_lawnmower.publish(msg_command_from_lawnmower);
    pub_command_from_lawnmower->publish(msg_command_from_lawnmower);
    //pub_from_lawnmower->publish(msg_command_from_lawnmower);
}


int main(int argc, char** argv){
    //ros::init(argc, argv, "lawnmower_can_topic_adapter");

    //ros::NodeHandle nh;

    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("lawnmower_can_topic_adapter");

    //ros::Subscriber sub_command_to_lawnmower = nh.subscribe("command_to_lawnmower", 10, callbackCaommandToLawnmower);
    //ros::Publisher pub_topic_to_socketcan = nh.advertise<can_msgs::Frame>("sent_messages", 10);
    auto sub_command_to_lawnmower = node->create_subscription<lawnmower_msgs::msg::CommandToLawnmower>("command_to_lawnmower", 10, callbackCaommandToLawnmower);
    //auto pub_topic_to_socketcan = node->create_publisher<can_msgs::msg::Frame>("sent_messages", 10);
    pub_topic_to_socketcan = node->create_publisher<can_msgs::msg::Frame>("sent_messages", 10);
    //auto pub_topic_to_socketcan = node->create_publisher<can_msgs::msg::Frame>("CAN/can0/transmit", 10);

    //ros::Subscriber sub_socketcan_to_topic = nh.subscribe("received_messages", 10, callbackSocketcanToTopic);
    //ros::Publisher pub_command_from_lawnmower = nh.advertise<lawnmower::command_from_lawnmower>("command_from_lawnmower", 10);
    auto sub_socketcan_to_topic = node->create_subscription<can_msgs::msg::Frame>("received_messages", 100, callbackSocketcanToTopic);
    //auto sub_socketcan_to_topic = node->create_subscription<can_msgs::msg::Frame>("CAN/can0/receive", 10, callbackSocketcanToTopic);
    //auto pub_command_from_lawnmower = node->create_publisher<lawnmower_msgs::msg::CommandFromLawnmower>("command_from_lawnmower", 10);
    pub_command_from_lawnmower = node->create_publisher<lawnmower_msgs::msg::CommandFromLawnmower>("command_from_lawnmower", 10);

    auto sub_grass_start = node->create_subscription<std_msgs::msg::Empty>("/orec/grass_start", 10, callbackGrassStart);
    auto sub_grass_stop = node->create_subscription<std_msgs::msg::Empty>("/orec/grass_stop", 10, callbackGrassStop);

    using namespace std::chrono_literals;
    auto timer = node->create_wall_timer(50ms, timerCallback);

    //ROS_INFO("Start CAN Topic Adapter");
    RCLCPP_INFO(node->get_logger(), "Start CAN Topic Adapter");

    rclcpp::spin(node);
    rclcpp::shutdown();

    /*
    //ros::Rate loop_rate(20);
    rclcpp::Rate loop_rate(20);

    //while(ros::ok()){
    while(rclcpp::ok()){

        //can_msgs::Frame msg_topic_to_socketcan;
        can_msgs::msg::Frame msg_topic_to_socketcan;
        //msg_topic_to_socketcan.id = 0x410;
        msg_topic_to_socketcan.id = 0x310;
        msg_topic_to_socketcan.dlc = 8;
        //msg_topic_to_socketcan.data = msg_data;
        msg_topic_to_socketcan.data = makeCommandData();
        //pub_topic_to_socketcan.publish(msg_topic_to_socketcan);
        pub_topic_to_socketcan->publish(msg_topic_to_socketcan);

        //lawnmower::command_from_lawnmower msg_command_from_lawnmower;
        lawnmower_msgs::msg::CommandFromLawnmower msg_command_from_lawnmower;
        msg_command_from_lawnmower.engine_speed = lawnmower_engine_speed;
        msg_command_from_lawnmower.speed_left = lawnmower_speed[0];
        msg_command_from_lawnmower.speed_right = lawnmower_speed[1];
        //pub_command_from_lawnmower.publish(msg_command_from_lawnmower);
        pub_command_from_lawnmower->publish(msg_command_from_lawnmower);

        //ros::spinOnce();
        rclcpp::spin_some(node);

        //loop_rate.sleep();
    }
    */
    
}
