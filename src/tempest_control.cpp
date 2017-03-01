/**
 * @file tempest_control.cpp
 * @A skeleton node provided as an example for how to operate an autonomous UAV. Currently requires extensive bug testing and improvement.
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/Waypoint.h>
#include <std_msgs/String.h>
#include <Tempest/TempestCmd.h> //this may or may not be the correct way of importing the tempest command msg, might need to be in a separate package
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>
#include <stdlib.h>

enum states {PREPARE,LAUNCH,LOITER,WAYPOINT,WAYPOINT_DONE,RTL,WAIT_LAND,LAND,HARD_LAND,SHUTDOWN};//states for the state machine
enum commands {TAKEOFF, STOP, GOTO_WAYPOINT, CANCEL, GO_LAND, GO_RTL, PING, NONE};//commands that can be recieved from SOLAR
commands current_command = NONE;//current command, held as global. Should probably make the entire thing a class as a result
mavros_msgs::State current_state;//current state of PixHawk, also a reason for fudamental change of code layout

//callback for PixHawk state publisher
void state_cb(const mavros_msgs::State::ConstPtr &msg){
    current_state = *msg;
}

//callback for higher level command (e.g. SOLAR) subscriber. The next command for the UAV comes in here.
void command_cb(tempest::TempestCmd &msg,mavros_msgs::CommandTOL &land_pose, mavros_msgs::CommandTOL &takeoff_pose, std::vector<mavros_msgs::Waypoint> &waypoints){
    tempest::TempestCmd incoming = msg->data;
    //deal with each command in enum commands separately, should be prety self explanitory
    if (incoming.command == TAKEOFF){
        takeoff_pose.request.latitude = incoming.lat;
        takeoff_pose.request.longitude = incoming.lng;
        takeoff_pose.request.altitude = incoming.alt;
        takeoff_pose.request.min_pitch = incoming.min_pitch;
        takeoff_pose.request.yaw = incoming.yaw;
        current_command = TAKEOFF;
    } else if (incoming.command == STOP){
        current_command = STOP;
    } else if (incoming.command == GOTO_WAYPOINT){
        mavros_msgs::Waypoint waypoint_command;
        waypoint_command.frame = 0;//2 for mission, 0 for global. not sure on context
        waypoint_command.command = 17;//16 for goto waypoint, 17 for loiter, 21 for land, 22 for takeoff
        waypoint_command.is_current = true;
        waypoint_command.autocontinue = true;
        waypoint_command.param1 = 0.0;//hold time at waypoint, copter and rover only
        waypoint_command.param2 = 5.0;//radius of waypoint acceptance, plane only
        waypoint_command.param3 = 0;//not really used, sets rotation around waypoint
        waypoint_command.param4 = 0;//copter only
        //can also trim using boost trim
        waypoint_command.x_lat = incoming.lat;
        waypoint_command.y_long = incoming.lng;
        waypoint_command.z_alt = incoming.alt;
        waypoints.push_back(waypoint_command);
        current_command = GOTO_WAYPOINT;
    } else if (incoming.command == CANCEL){
        //should result in all waypoints being cancelled and going into a loiter mode
        current_command = CANCEL;
    } else if (incoming.command == PING){
        //higher level system should periodically send a ping to show that it is still working. Current code setup means that this will be missed
        current_command = PING;
    } else if (incoming.command == GO_LAND){
        land_pose.request.latitude = incoming.lat;
        land_pose.request.longitude = incoming.lng;
        land_pose.request.altitude = incoming.alt;
        land_pose.request.min_pitch = incoming.min_pitch;
        land_pose.request.yaw = incoming.yaw;
        current_command = GO_LAND;
    } else if (incoming.command == GO_RTL){
        //command given to return to launch site, or other defined location
        land_pose.request.latitude = incoming.lat;
        land_pose.request.longitude = incoming.lng;
        land_pose.request.altitude = incoming.alt;
        land_pose.request.min_pitch = incoming.min_pitch;
        land_pose.request.yaw = incoming.yaw;
        current_command = GO_RTL;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "tempest_node");//say hi to ROS
    ros::NodeHandle nh;//tempest node handle
    mavros_msgs::CommandTOL land_pose;//hold the currently designated land site
    mavros_msgs::CommandTOL takeoff_pose;//hold designed takeoff site
    std::vector<mavros_msgs::Waypoint> waypoints;//list of waypoints to be achieved, as a list. Should work through this list.

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);//state of PixHawk
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);//only really here to help with initialisation
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");//arming service
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");//mopde change service
    ros::ServiceClient waypoint_client = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");//send a new mission, i.e. waypoint list
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");//send a land command
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");//send a takeoff command
    ros::Subscriber command_sub = nh.subscribe<tempest::TempestCmd>("tempest/to_front_seat",1000,boost::bind(command_cb,_1,land_pose,takeoff_pose,waypoints));//incoming messages from autonomy system

    

    //main control loop, holds the state machine
    states current = PREPARE;
    bool armed = false;
    bool launched = false;
    bool land_permission = false;
    bool launch_request_sent = false;
    bool waypoint_sent = false;
    ros::Time last_request = ros::Time::now();
    ros::Time last_ping = ros::Time::now();
    int nep_fail_count = 0;
    mavros_msgs::CommandTOL launch_command;//prep command to launch
    mavros_msgs::WaypointPush send_waypoint;//prep command to send new waypoint
    mavros_msgs::SetMode offb_set_mode;//mode set command object
    offb_set_mode.request.custom_mode = "OFFBOARD";//set mode
    mavros_msgs::CommandBool arm_cmd;//boolean command object
    arm_cmd.request.value = true;//set arm command
    float alt_hold = 30.0;//m
    nh.setParam("min_flight_alt", alt_hold);//stores in ROS params list, for easy setting
    ros::Rate rate(20.0);
    
    while(ros::ok() && current_state.connected){//bit counter intuitive, current_state.commected returns a true when it isn't
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting, just to ensure the connection
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
    while(ros::ok()){
        //check for recent ping from neptune, log if it hasn't connected' currently a silly way of doing it. Maybe do a separate ping subscriber
//        if ((ros::Time::now() - last_ping) > ros::Duration(5.0)) {
//           ROS_INFO("neptune not connected??");
//           nep_fail_count = nep_fail_count + 1;
//        }
        //RTL if neptune has failed
        if (nep_fail_count > 3) {
            current = RTL;
            ROS_INFO("Neptune conection failed, returning to designated landing site");
        }
        //switch back to waypoint if we have one and it is asked for 
        if (current_command == GOTO_WAYPOINT){
            ROS_INFO("waypoint command recieved");
            current = WAYPOINT;
        }

        switch(current) {
            //connect to the pixhawk, ready for takeoff
            case PREPARE:
                if (armed == false){
                    //if not in offboard mode and we have asked more than 5 seconds ago, then attempt to set it
                    if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        //send arm command and then check it
                        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){
                            ROS_INFO("Offboard enabled");
                        }
                        last_request = ros::Time::now();
                    }
                        
                }
                else {
                    if(ros::Time::now() - last_request > ros::Duration(5.0)){
                        ROS_INFO("Vehicle prepared");
                        last_request = ros::Time::now();
                    }
                    
                    if (current_command == TAKEOFF) {
                        current = LAUNCH;
                        ROS_INFO("Changing to launch mode");
                    }
                }
                break;
            //when a launch command is recieved, arm and launch    
            case LAUNCH:
                if (launch_request_sent == false) {
                    if(!current_state.armed){
                            //try arming and check
                            if( arming_client.call(arm_cmd) &&
                                arm_cmd.response.success){
                                ROS_INFO("Vehicle armed");
                                armed = true;
                            }
                            last_request = ros::Time::now();
                    }
                    if(current_state.armed){
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
                    }
                } else {
                    if (launch_command.response.success) {
                        ROS_INFO("Vehicle launched, loitering");
                        current = LOITER;
                    } else {
                        ROS_INFO("Vehicle still launching");
                        //maybe at this point also consider a failure mode if launch was not successful
                    }
                    last_request = ros::Time::now();
                }
                break;
            //go into loiter mode
            case LOITER:
                    if( current_state.mode != "AUTO.LOITER" &&
                            (ros::Time::now() - last_request > ros::Duration(5.0))){
                            mavros_msgs::SetMode loiter_set_mode;//mode set command object
                            offb_set_mode.request.custom_mode = "AUTO.LOITER";//set mode
                            //send arm command and then check it
                            if( set_mode_client.call(loiter_set_mode) &&
                                offb_set_mode.response.success){
                                ROS_INFO("loiter enabled");
                            }
                            last_request = ros::Time::now();
                    }
                break;
            //go to the next waypoint
            case WAYPOINT:
                if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        //send offbaord command and then check it
                        if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.success){
                            ROS_INFO("Offboard re-enabled");
                        }
                        last_request = ros::Time::now();
                }
                if (waypoint_sent == false) {
                    send_waypoint.request.waypoints = waypoints; //sends the entire list of waypoints
                    if(current_state.mode == "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if (waypoint_client.call(send_waypoint)){
                            ROS_INFO("new waypoint sent");
                            launch_request_sent = true;
                        }
                        last_request = ros::Time::now();
                    }
                } else {
                    if (send_waypoint.response.success) {
                        ROS_INFO("arrived at waypoint, waiting");
                        current = LOITER;
                    } else {
                        ROS_INFO("Vehicle in transit");
                        //maybe at this point also consider a failure mode if launch was not successful
                    }
                    last_request = ros::Time::now();
                }
                break;
            case RTL:
                    //send home position as waypoint, then loiter
                break;
            case WAIT_LAND:
                    //a loiter mode. wait here until the battery runs too low, then crash somehow

                break;
            case LAND:
                    //send a land command once confirmation is recieved
                break;
            case HARD_LAND:
                    //crash somehow
                break;
            case SHUTDOWN:
                    //disarm and shut down
                break;
        }
        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
