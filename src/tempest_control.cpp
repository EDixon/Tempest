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
#include <mavros_msgs/CommandTOL.h>
#include <std_msgs/String.h>
#include <string>
#include <boost/algorithm/string.hpp>

enum states {PREPARE,LAUNCH,LOITER,WAYPOINT,WAYPOINT_DONE,RTL,WAIT_LAND,LAND,HARD_LAND,COM_CHECK,SHUTDOWN};
enum commands {START, STOP, WAYPOINT, CANCEL, LAND, RTL}


class waypoint{
    public:
        double lat;
        double lng;
        double alt;
        double speed;
        double heading;
        int waypoint_n
        command();
};

waypoint::waypoint(void){
    lat = 0.0;
    lng = 0.0;
    alt = 0.0;
    speed = 0.0;
    heading = 0.0;
    waypoint_n = 0;
}

class flight_pose{
    public:
        double lat;
        double lng;
        double alt;
        double speed;
        double heading;
        int pose_n
        flight_pose();
        copy_waypoint(waypoint &waypoint_in)
};

flight_pose::flight_pose(void){
    lat = 0.0;
    lng = 0.0;
    alt = 0.0;
    speed = 0.0;
    heading = 0.0;
    pose_n= 0;
}

void flight_pose::copy_command(command command_in){
    lat = waypoint_in.lat;
    lng = waypoint_in.lng;
    alt = waypoint_in.alt;
    speed = waypoint_in.speed;
    heading = waypoint_in.heading;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

std:vector<std::string> split(const std::string &line){
    std::vector<std::string> parts;
    boost::split(parts)
}


ros::Time last_ping = ros::Time::now();
void command_cb(const std_msgs::String::ConstPtr &msg,flight_pose &land_pose, flight_pose &takeoff_pose, std::vector<waypoint> &waypoints){
    std::string incoming = msg->data;
    if (incoming.compare(0,7,"$START,") == 0){
        current_command = START;
    } else if (incoming.compare(0,6,"$STOP,") == 0){
        current_command = STOP;
    } else if (incoming.compare(0,10,"$WAYPOINT,") == 0){
        std::vector<std::string> parts;
        boost::parts(parts, incoming, boost::is_any_of(","));
        waypoint waypoint_hold;
        waypoint_hold.lat = std::stod(parts.pos(1),0);
        waypoint_hold.lng = std::stod(parts.pos(2),0);
        waypoint_hold.alt = std::stod(parts.pos(2),0);
        waypoint_hold.speed = std::stop(parts.pos(3),0);
        waypoint_hold.heading = std::stod(parts.pos(4),0);
        waypoint_hold.waypoint_n = int(std::stod(parts.pos(5),0));
        //might have to check to see if waypoints[0] exists first
        if (waypoints.back()).waypoint_n < waypoint_hold.waypoint_n){
            waypoints.push_back(waypoint_hold);
        }
        current_command = WAYPOINT;
    } else if (incoming.compare(0,8,"$CANCEL,") == 0){
        current_command = CANCEL;
    } else if (incoming.compare(0,6,"$PING,") == 0){
        last_ping = ros::Time::now();
    } else if (incoming.compare(0,10,"$LAND,") == 0){
        std::vector<std::string> parts;
        boost::parts(parts, incoming, boost::is_any_of(","));
        land_pose.lat = std::stod(parts.pos(1),0);
        land_pose.lng = std::stod(parts.pos(2),0);
        land_pose.alt = std::stod(parts.pos(2),0);
        land_pose.speed = std::stop(parts.pos(3),0);
        land_pose.heading = std::stod(parts.pos(4),0);
        land_pose.pose_n = int(std::stod(parts.pos(5),-));
        current_command = LAND;
    }
}

commands current_command;
int main(int argc, char **argv)


    ros::init(argc, argv, "tempest_node");
    ros::NodeHandle nh;
    flight_pose land_pose;
    flight_pose takeoff_pose;
    std::vector<waypoint> waypoints;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros:Subscriber command_sub = nh.subscribe<std_msgs::String>("to_front_seat",1000,boost::bind(command_cb,_1,land_pose,takeoff_pose,waypoints))

    ros::Rate rate(20.0);

    //main control loop, holds the state machine
    states current = PREPARE;
    bool armed = false;
    bool launched = false;
    bool land_permission = false;
    bool launch_request_sent = false;
    bool have_waypoint = false;
    ros::Time last_request = ros::Time::now();
    int nep_fail_count = 0;
    flight_pose takeoff_config;
    flight_pose landing_config;
    std::float alt_hold = 10.0;
    nh.setParam("min_flight_alt", alt_hold);
    while(ros::ok()){
        //check for recent ping from neptune, log if it hasn't connected'
        if (ros::Time::now() - last_ping) > 5.0{
            ROS_INFO("neptune not connected??");
            nep_fail_count = nep_fail_count + 1;
        }
        //RTL if neptune has failed
        if nep_fail_count > 3 {
            current = RTL;
        }
        //switch back to waypoint if we have one and it is asked for 
        if waypoint.size() > 0 && current_command == WAYPOINT {
            current = WAYPOINT;
        }

        switch(current) {
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
                    if current_command.state == START {
                        current = LAUNCH;
                    }
                }
                break;
            case LAUNCH:
                if launch_request_sent == false {
                    mavros_msgs::CommandTOL launch_command;
                    launch_command.request.min_pitch =  float(0.0);//only used at takeoff
                    launch_command.request.yaw = float(0.0);
                    launch_command.request.latitude = float(0.0);
                    launch_command.request.longitude = float(0.0);
                    nh.getParam("min_flight_alt", alt_hold);
                    launch_command.request.altitude = float(alt_hold);
                    if(current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if (takeoff_client.call(launch_command)){
                            ROS_INFO("Launch command sent");
                            launch_request_sent = true;
                        }
                        last_request = ros::Time::now();
                    }
                } else {
                    if launch_command.response.success {
                        ROS_INFO("Vehicle launched, loitering")
                        current = LOITER;
                    } else {
                        ROS_INFO("Vehicle still launching")
                        //maybe at this point also consider a failure mode if launch was not successful
                    }
                    last_request = ros::Time::now();
                }
                

                break;
            case LOITER:
                    if( current_state.mode != "AUTO.LOITER" &&
                            (ros::Time::now() - last_request > ros::Duration(5.0))){
                            mavros_msgs::SetMode loiter_set_mode;//mode set command object
                            offb_set_mode.request.custom_mode = "AUTO.LOITER"//set mode
                            //send arm command and then check it
                            if( set_mode_client.call(loiter_set_mode) &&
                                offb_set_mode.response.success){
                                ROS_INFO("loiter enabled");
                            }
                            last_request = ros::Time::now();
                    }

                break;
            case WAYPOINT:
                waypoint go_here;
                go_here = waypoints.front();
                //lookup mavros waypoint type, might be a good idea to steal that 
                //might have to use the PixHawks own waypoint management to add and remove waypoints. 

                break;
            case WAYPOINT_DONE:
                //remove waypoint from list

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
