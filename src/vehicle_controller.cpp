#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "lawnmower/command_to_lawnmower.h"
#include "lawnmower/command_from_lawnmower.h"

const double distance_per_rot = 14.06; // モーター1回転で進む距離[mm]


// cmd_velに対するコールバック
geometry_msgs::Twist cmd_vel;
void callbackCmdVel(const geometry_msgs::Twist::ConstPtr& msg){
    cmd_vel.linear = msg->linear;
    cmd_vel.angular = msg->angular;
}

// cmd_velからLawnMowerに送るコマンドを作る
lawnmower::command_to_lawnmower makeCommandToLawnmower(){
    lawnmower::command_to_lawnmower msg;
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