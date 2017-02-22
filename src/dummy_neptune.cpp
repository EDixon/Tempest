#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>


class tempest_controller{
   public:
    tempest_controller();
    void takeoff();
   private:
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub;
};

tempest_controller::tempest_controller(){
    local_pos_pub  = nh.advertise<std_msgs::String>("tempest/to_front_seat", 10);
}

void tempest_controller::takeoff(void){
        
        ros::Rate loop_rate(10);
        int count = 0;
        while (ros::ok()){
            std_msgs::String str;
            str.data = "$TAKEOFF,0,0,50,0,0";
            ROS_INFO("%s",str.data.c_str());
            local_pos_pub.publish(str);
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