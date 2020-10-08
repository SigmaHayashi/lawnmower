#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"

const double distance_per_rot = 14.06 / 1000; // モーター1回転で進む距離[m]
const double distance_wheel = 0.75;            // 左右輪の距離[m]

// cmd_velに対するコールバック
geometry_msgs::Twist cmd_vel;
void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_vel.linear = msg->linear;
    cmd_vel.angular = msg->angular;

    //cmd_vel.linear.x *= 0.5;
    //cmd_vel.angular.z *= 0.5;
}

// cmd_velからLawnMowerに送るコマンドを作る
lawnmower::command_to_lawnmower makeCommandToLawnmower(){
    lawnmower::command_to_lawnmower msg; // [m/sec] [rad/min]

    int cmd_vel_rpm[2];
    cmd_vel_rpm[0] = (cmd_vel.linear.x - distance_wheel / 2 * cmd_vel.angular.z) * 60 / distance_per_rot;
    cmd_vel_rpm[1] = (cmd_vel.linear.x + distance_wheel / 2 * cmd_vel.angular.z) * 60 / distance_per_rot;
    ROS_INFO("cmd_vel_rpm : %d %d", cmd_vel_rpm[0], cmd_vel_rpm[1]);

    if(cmd_vel_rpm[0] > 0){
        if(cmd_vel_rpm[0] >= (2013 + 1602) / 2){
            msg.speed_left = 5;
        }
        else if(cmd_vel_rpm[0] >= (1602 + 1109) / 2){
            msg.speed_left = 4;
        }
        else if(cmd_vel_rpm[0] >= (1109 + 673) / 2){
            msg.speed_left = 3;
        }
        else if(cmd_vel_rpm[0] >= (673 + 268) / 2){
            msg.speed_left = 2;
        }
        else if(cmd_vel_rpm[0] >= 268 / 2){
            msg.speed_left = 1;
        }
        else{
            msg.speed_left = 0;
        }
    }
    else if(cmd_vel_rpm[0] < 0){
        if(cmd_vel_rpm[0] <= (2013 + 1602) / -2){
            msg.speed_left = -5;
        }
        else if(cmd_vel_rpm[0] <= (1602 + 1109) / -2){
            msg.speed_left = -4;
        }
        else if(cmd_vel_rpm[0] <= (1109 + 673) / -2){
            msg.speed_left = -3;
        }
        else if(cmd_vel_rpm[0] <= (673 + 268) / -2){
            msg.speed_left = -2;
        }
        else if(cmd_vel_rpm[0] <= 268 / -2){
            msg.speed_left = -1;
        }
        else{
            msg.speed_left = 0;
        }
    }
    else{
        msg.speed_left = 0;
    }

    if(cmd_vel_rpm[1] > 0){
        if(cmd_vel_rpm[1] >= (2013 + 1602) / 2){
            msg.speed_right = 5;
        }
        else if(cmd_vel_rpm[1] >= (1602 + 1109) / 2){
            msg.speed_right = 4;
        }
        else if(cmd_vel_rpm[1] >= (1109 + 673) / 2){
            msg.speed_right = 3;
        }
        else if(cmd_vel_rpm[1] >= (673 + 268) / 2){
            msg.speed_right = 2;
        }
        else if(cmd_vel_rpm[1] >= 268 / 2){
            msg.speed_right = 1;
        }
        else{
            msg.speed_right = 0;
        }
    }
    else if(cmd_vel_rpm[1] < 0){
        if(cmd_vel_rpm[1] <= (2013 + 1602) / -2){
            msg.speed_right = -5;
        }
        else if(cmd_vel_rpm[1] <= (1602 + 1109) / -2){
            msg.speed_right = -4;
        }
        else if(cmd_vel_rpm[1] <= (1109 + 673) / -2){
            msg.speed_right = -3;
        }
        else if(cmd_vel_rpm[1] <= (673 + 268) / -2){
            msg.speed_right = -2;
        }
        else if(cmd_vel_rpm[1] <= 268 / -2){
            msg.speed_right = -1;
        }
        else{
            msg.speed_right = 0;
        }
    }
    else{
        msg.speed_right = 0;
    }

    return msg;
}


// 草刈り機の状態を記録しておく
lawnmower::command_from_lawnmower var_command_from_lawnmower;
void callbackCommandFromLawnmower(const lawnmower::command_from_lawnmower::ConstPtr& msg){
    var_command_from_lawnmower = *msg;
}


// メイン関数
int main(int argc, char** argv){

    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ROS_INFO("Vehicle Controller Start");

    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 10, callbackCmdVel);
    ros::Publisher pub_command_to_lawnmower = nh.advertise<lawnmower::command_to_lawnmower>("command_to_lawnmower", 10);

    ros::Subscriber sub_command_from_lawnmofer = nh.subscribe("command_from_lawnmower", 10, callbackCommandFromLawnmower);
    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Rate loop_rate(20);

    // メインループ
    while(ros::ok()){
        pub_command_to_lawnmower.publish(makeCommandToLawnmower());

        double lawnmower_speed_mps[2];
        lawnmower_speed_mps[0] = var_command_from_lawnmower.speed_left * distance_per_rot / 60;
        lawnmower_speed_mps[1] = var_command_from_lawnmower.speed_right * distance_per_rot / 60;

        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.twist.twist.linear.x = (lawnmower_speed_mps[0] + lawnmower_speed_mps[1]) / 2;
        odom.twist.twist.angular.z = (lawnmower_speed_mps[1] - lawnmower_speed_mps[0]) / distance_wheel;
        pub_odom.publish(odom);
        
        ros::spinOnce();

        loop_rate.sleep();
    }

}