/**
 * @file dummy_neptune.cpp
 * @For testing, emulating what the neptune node has to do to command the tempest node
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <tempest/TempestCmd>//still not convinced this is the right way of doing it.
enum commands {TAKEOFF, STOP, GOTO_WAYPOINT, CANCEL, GO_LAND, GO_RTL, PING, NONE};

class tempest_controller{
   public:
    tempest_controller();
    void takeoff();
   private:
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub;
};

tempest_controller::tempest_controller(){
    local_pos_pub  = nh.advertise<tempest::TempestCmd>("tempest/to_front_seat", 10);
}

void tempest_controller::takeoff(void){
        
        ros::Rate loop_rate(10);
        tempest::TempestCmd send_command;
        send_command.data.command = TAKEOFF:

        send_command.data.lat = 0.0;
        send_command.data.lng = 0.0;
        send_command.data.alt = 50.0;
        send_command.data.min_pitch = 0.0;
        send_command.data.yaw = 0.0;
        int count = 0;
        while (ros::ok()){
            local_pos_pub.publish(send_comand);
            ros::spinOnce();
            loop_rate.sleep();
            ++count;
        }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dummy_neptune");

    tempest_controller control;
    control.takeoff();
    ros::spin();
    return 0;
}