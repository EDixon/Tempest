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
#include <std_msgs/String.h>
#include <string>
#include <boost/algorithm/string.hpp>

enum states {PREPARE,LAUNCH,LOITER,WAYPOINT,WAYPOINT_DONE,RTL,WAIT_LAND,LAND,HARD_LAND,COM_CHECK,SHUTDOWN};
enum commands {START, STOP, WAYPOINT, CANCEL, PING, LAND}


class command{
    public:
        commands state;
        double lat;
        double lng;
        double alt;
        double speed;
        double heading;
        int waypoint_n
        command();
};

command::command(void){
    lat = 0.0;
    lng = 0.0;
    alt = 0.0;
    speed = 0.0;
    heading = 0.0;
    waypoint_n = 0;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

std:vector<std::string> split(const std::string &line){
    std::vector<std::string> parts;
    boost::split(parts)
}

command current_command;
void command_cb(const std_msgs::String::ConstPtr &msg){
    std::string incoming = msg->data;
    if (incoming.compare(0,7,"$START,") == 0){
        current_command.state = START;
    } else if (incoming.compare(0,6,"$STOP,") == 0){
        current_command.state = STOP;
    } else if (incoming.compare(0,10,"$WAYPOINT,") == 0){
        std::vector<std::string> parts;
        
        
        current_command.state = WAYPOINT;
    } else if (incoming.compare(0,8,"$CANCEL,") == 0){
        current_command.state = CANCEL;
    } else if (incoming.compare(0,6,"$PING,") == 0){
        current_command.state = PING;
    } else if (incoming.compare(0,10,"$LAND,") == 0){
        current_command.state = LAND;
    }
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
    ros:Subscriber command_sub = nh.subscribe<std_msgs::String>("to_front_seat",1000,command_cb)
    
    ros::Rate rate(20.0);

    //main control loop, holds the state machine
    states current = PREPARE;
    bool armed = false;
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        switch(current) {
            //check that the pixhawk is still connected
            case COM_CHECK:
                
                break;
            //connect to the pixhawk and arm
            case PREPARE:
                if (armed == false){
                    mavros_msgs::SetMode offb_set_mode;//mode set command object
                    offb_set_mode.request.custom_mode = "OFFBOARD"//set mode
                    mavros_msgs::CommandBool arm_cmd;//boolean command object
                    arm_cmd.request.value = true;//set arm command
                    //if not in offboard mode and we have asked more than 5 seconds ago, then attempt to set it
                    if( current_state.mode != "OFFBOARD" &&
                        (ros::Time::now() - last_request > ros::Duration(5.0))){
                        //send arm command and then check it
                        if( set_mode_client.call(offb_set_mode) &&
                            offb_set_mode.response.success){
                            ROS_INFO("Offboard enabled");
                        }
                        last_request = ros::Time::now();
                    //if we are in offboard mode, try arming
                    } else {
                        if( !current_state.armed &&
                            (ros::Time::now() - last_request > ros::Duration(5.0))){
                            //try arming and check
                            if( arming_client.call(arm_cmd) &&
                                arm_cmd.response.success){
                                ROS_INFO("Vehicle armed");
                                armed == true;
                            }
                            last_request = ros::Time::now();
                        }
                    }
                }
                else {
                    ROS_INFO("Vehicle prepared");
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
