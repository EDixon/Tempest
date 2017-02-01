/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
enum states {PREPARE,LAUNCH,LOITER,WAYPOINT,WAYPOINT_DONE,RTL,WAIT_LAND,LAND,HARD_LAND,COM_CHECK,SHUTDOWN};

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)


    ros::init(argc, argv, "tempest_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    
    ros::Rate rate(20.0);

    //main control loop, holds the state machine
    states current = PREPARE;
    bool connected = false;
    while(ros::ok()){
        switch(current) {
            case COM_CHECK:

                break;
            case PREPARE:
                if (connected == false){
                    mavros_msgs::SetMode offb_set_mode;
                    offb_set_mode.request.custom_mode = "OFFBOARD"
                    mavros_msgs::CommandBool arm_cmd;
                    arm_cmd.request.value = true;
                }
                break;
            case LAUNCH:

                break;
            case LOITER:

                break;
            case WAYPOINT:

                break;
            case WAYPOINT_DONE:

                break;
            case RTL:

                break;
            case WAIT_LAND:

                break;
            case LAND:

                break;
            case HARD_LAND:

                break;
            case SHUTDOWN:

                break;
        }





        ros::spinOnce();
        rate.sleep();
    }
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}