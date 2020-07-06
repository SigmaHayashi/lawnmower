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
const double distance_wheel = 1.0;  // 左右輪の距離[m]

// cmd_velに対するコールバック
geometry_msgs::Twist cmd_vel;
void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_vel.linear = msg->linear;
    cmd_vel.angular = msg->angular;
}

// cmd_velからLawnMowerに送るコマンドを作る
lawnmower::command_to_lawnmower makeCommandToLawnmower(){
    lawnmower::command_to_lawnmower msg; // [m/sec] [rad/min]
    /*
    if(cmd_vel.angular.z == 0){
        if(cmd_vel.linear.x > 1.0){
            msg.speed_left = 4;
            msg.speed_right = 4;
        }
        else if(cmd_vel.linear.x > 0.5){
            msg.speed_left = 2;
            msg.speed_right = 2;
        }
        else if(cmd_vel.linear.x < -1.0){
            msg.speed_left = -4;
            msg.speed_right = -4;
        }
        else if(cmd_vel.linear.x < -0.5){
            msg.speed_left = -2;
            msg.speed_right = -2;
        }
        else{
            msg.speed_left = 0;
            msg.speed_right = 0;
        }
    }
    else{
        if(cmd_vel.linear.x == 0){
            if(cmd_vel.angular.z > 0){
                msg.speed_left = -4;
                msg.speed_right = 4;
            }
            else if(cmd_vel.angular.z < 0){
                msg.speed_left = 4;
                msg.speed_right = -4;
            }
        }
    }
    */

    //int cmd_vel_rpm = cmd_vel.linear.x * 60 / (distance_per_rot / (2 * M_PI));
    int cmd_vel_rpm_left = (cmd_vel.linear.x - distance_wheel / 2 * cmd_vel.angular.z) * 60 / (distance_per_rot / (2 * M_PI));
    int cmd_vel_rpm_right = (cmd_vel.linear.x + distance_wheel / 2 * cmd_vel.angular.z) * 60 / (distance_per_rot / (2 * M_PI));
    //ROS_INFO("cmd_vel_rpm : %d %d %d", cmd_vel_rpm, cmd_vel_rpm_left, cmd_vel_rpm_right);
    ROS_INFO("cmd_vel_rpm : %d %d", cmd_vel_rpm_left, cmd_vel_rpm_right);

    if(cmd_vel_rpm_left > 0){
        if(cmd_vel_rpm_left >= 2013){
            msg.speed_left = 5;
        }
        else if(cmd_vel_rpm_left >= 1602){
            msg.speed_left = 4;
        }
        else if(cmd_vel_rpm_left >= 1109){
            msg.speed_left = 3;
        }
        else if(cmd_vel_rpm_left >= 673){
            msg.speed_left = 2;
        }
        else if(cmd_vel_rpm_left >= 268){
            msg.speed_left = 1;
        }
        else{
            msg.speed_left = 0;
        }
    }
    else if(cmd_vel_rpm_left < 0){
        if(cmd_vel_rpm_left <= -2013){
            msg.speed_left = -5;
        }
        else if(cmd_vel_rpm_left <= -1602){
            msg.speed_left = -4;
        }
        else if(cmd_vel_rpm_left <= -1109){
            msg.speed_left = -3;
        }
        else if(cmd_vel_rpm_left <= -673){
            msg.speed_left = -2;
        }
        else if(cmd_vel_rpm_left <= -268){
            msg.speed_left = -1;
        }
        else{
            msg.speed_left = 0;
        }
    }
    else{
        msg.speed_left = 0;
    }

    if(cmd_vel_rpm_right > 0){
        if(cmd_vel_rpm_right >= 2013){
            msg.speed_right = 5;
        }
        else if(cmd_vel_rpm_right >= 1602){
            msg.speed_right = 4;
        }
        else if(cmd_vel_rpm_right >= 1109){
            msg.speed_right = 3;
        }
        else if(cmd_vel_rpm_right >= 673){
            msg.speed_right = 2;
        }
        else if(cmd_vel_rpm_right >= 268){
            msg.speed_right = 1;
        }
        else{
            msg.speed_right = 0;
        }
    }
    else if(cmd_vel_rpm_right < 0){
        if(cmd_vel_rpm_right <= -2013){
            msg.speed_right = -5;
        }
        else if(cmd_vel_rpm_right <= -1602){
            msg.speed_right = -4;
        }
        else if(cmd_vel_rpm_right <= -1109){
            msg.speed_right = -3;
        }
        else if(cmd_vel_rpm_right <= -673){
            msg.speed_right = -2;
        }
        else if(cmd_vel_rpm_right <= -268){
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


// メイン関数
int main(int argc, char** argv){

    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;

    ROS_INFO("Vehicle Controller Start");

    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 1, callbackCmdVel);
    ros::Publisher pub_command_to_lawnmower = nh.advertise<lawnmower::command_to_lawnmower>("command_to_lawnmower", 1);

    ros::Rate loop_rate(20);

    // メインループ
    while(ros::ok()){
        pub_command_to_lawnmower.publish(makeCommandToLawnmower());
        
        ros::spinOnce();

        loop_rate.sleep();
    }

}