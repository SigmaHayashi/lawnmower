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


// cmd_vel -> command_to_lawnmower -> command_from_lawnmower -> odom
// の過程をすっ飛ばす
nav_msgs::Odometry odom;
void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear = msg->linear;
    cmd_vel.angular = msg->angular;

    int cmd_vel_rpm[2];
    cmd_vel_rpm[0] = (cmd_vel.linear.x - distance_wheel / 2 * cmd_vel.angular.z) * 60 / distance_per_rot;
    cmd_vel_rpm[1] = (cmd_vel.linear.x + distance_wheel / 2 * cmd_vel.angular.z) * 60 / distance_per_rot;
    ROS_INFO("cmd_vel_rpm : %d %d", cmd_vel_rpm[0], cmd_vel_rpm[1]);

    int speed_left_simple = 0;
    int speed_right_simple = 0;
    int speed_left = 0;
    int speed_right = 0;
    if(cmd_vel_rpm[0] > 0){
        if(cmd_vel_rpm[0] >= (2013 + 1602) / 2){
            //msg.speed_left = 5;
            speed_left_simple = 5;
            speed_left = 2013;
        }
        else if(cmd_vel_rpm[0] >= (1602 + 1109) / 2){
            //msg.speed_left = 4;
            speed_left_simple = 4;
            speed_left = 1602;
        }
        else if(cmd_vel_rpm[0] >= (1109 + 673) / 2){
            //msg.speed_left = 3;
            speed_left_simple = 3;
            speed_left = 1109;
        }
        else if(cmd_vel_rpm[0] >= (673 + 268) / 2){
            //msg.speed_left = 2;
            speed_left_simple = 2;
            speed_left = 673;
        }
        //else if(cmd_vel_rpm[0] >= 268 / 2){
        else if(cmd_vel_rpm[0] > 0){
            //msg.speed_left = 1;
            speed_left_simple = 1;
            speed_left = 268;
        }
        else{
            //msg.speed_left = 0;
            speed_left_simple = 0;
            speed_left = 0;
        }
    }
    else if(cmd_vel_rpm[0] < 0){
        if(cmd_vel_rpm[0] <= (2013 + 1602) / -2){
            //msg.speed_left = -5;
            speed_left_simple = -5;
            speed_left = -2013;
        }
        else if(cmd_vel_rpm[0] <= (1602 + 1109) / -2){
            //msg.speed_left = -4;
            speed_left_simple = -4;
            speed_left = -1602;
        }
        else if(cmd_vel_rpm[0] <= (1109 + 673) / -2){
            //msg.speed_left = -3;
            speed_left_simple = -3;
            speed_left = -1109;
        }
        else if(cmd_vel_rpm[0] <= (673 + 268) / -2){
            //msg.speed_left = -2;
            speed_left_simple = -2;
            speed_left = -673;
        }
        //else if(cmd_vel_rpm[0] <= 268 / -2){
        else if(cmd_vel_rpm[0] < 0){
            //msg.speed_left = -1;
            speed_left_simple = -1;
            speed_left = -268;
        }
        else{
            //msg.speed_left = 0;
            speed_left_simple = 0;
            speed_left = 0;
        }
    }
    else{
        //msg.speed_left = 0;
        speed_left_simple = 0;
        speed_left = 0;
    }

    if(cmd_vel_rpm[1] > 0){
        if(cmd_vel_rpm[1] >= (2013 + 1602) / 2){
            //msg.speed_right = 5;
            speed_right_simple = 5;
            speed_right = 2013;
        }
        else if(cmd_vel_rpm[1] >= (1602 + 1109) / 2){
            //msg.speed_right = 4;
            speed_right_simple = 4;
            speed_right = 1602;
        }
        else if(cmd_vel_rpm[1] >= (1109 + 673) / 2){
            //msg.speed_right = 3;
            speed_right_simple = 3;
            speed_right = 1109;
        }
        else if(cmd_vel_rpm[1] >= (673 + 268) / 2){
            //msg.speed_right = 2;
            speed_right_simple = 2;
            speed_right = 673;
        }
        //else if(cmd_vel_rpm[1] >= 268 / 2){
        else if(cmd_vel_rpm[1] > 0){
            //msg.speed_right = 1;
            speed_right_simple = 1;
            speed_right = 268;
        }
        else{
            //msg.speed_right = 0;
            speed_right_simple = 0;
            speed_right = 0;
        }
    }
    else if(cmd_vel_rpm[1] < 0){
        if(cmd_vel_rpm[1] <= (2013 + 1602) / -2){
            //msg.speed_right = -5;
            speed_right_simple = -5;
            speed_right = -2013;
        }
        else if(cmd_vel_rpm[1] <= (1602 + 1109) / -2){
            //msg.speed_right = -4;
            speed_right_simple = -4;
            speed_right = -1602;
        }
        else if(cmd_vel_rpm[1] <= (1109 + 673) / -2){
            //msg.speed_right = -3;
            speed_right_simple = -3;
            speed_right = -1109;
        }
        else if(cmd_vel_rpm[1] <= (673 + 268) / -2){
            //msg.speed_right = -2;
            speed_right_simple = -2;
            speed_right = -673;
        }
        //else if(cmd_vel_rpm[1] <= 268 / -2){
        else if(cmd_vel_rpm[1] < 0){
            //msg.speed_right = -1;
            speed_right_simple = -1;
            speed_right = -268;
        }
        else{
            //msg.speed_right = 0;
            speed_right_simple = 0;
            speed_right = 0;
        }
    }
    else{
        //msg.speed_right = 0;
        speed_right_simple = 0;
        speed_right = 0;
    }

    ROS_INFO("Command Speed: %d, %d", speed_left_simple, speed_right_simple);

    double speed_left_mps = speed_left * distance_per_rot / 60;
    double speed_right_mps = speed_right * distance_per_rot / 60;

    odom.twist.twist.linear.x = (speed_left_mps + speed_right_mps) / 2;
    odom.twist.twist.angular.z = (speed_right_mps - speed_left_mps) / distance_wheel;
    odom.twist.twist.angular.z *= 0.5;
}


// メイン関数
int main(int argc, char** argv){

    ros::init(argc, argv, "virtual_vehicle_controller");
    ros::NodeHandle nh;

    ROS_INFO("Virtual Vehicle Controller Start");

    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, callbackCmdVel);

    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Rate loop_rate(20);

    // メインループ
    while(ros::ok()){
        odom.header.stamp = ros::Time::now();
        pub_odom.publish(odom); // odoomをパブリッシュ
        
        ros::spinOnce();

        loop_rate.sleep();
    }

}