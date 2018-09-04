#include <ros/ros.h>
#include <hector_uav_msgs/EnableMotors.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <sys/select.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <csignal>


double velocity_multiplier = 1.0;
void velocity_commander(std::string& command,geometry_msgs::Twist& msg){
    char comm_descriptor;
    if(command.length()!=1)
        comm_descriptor = '\0';
    else
        comm_descriptor = command[0];
    
    switch(comm_descriptor){
        case 'a':
            msg.linear.x = 1.0*velocity_multiplier;
            break;
        case 'd':
            msg.linear.x = -1.0*velocity_multiplier;
            break;
        case 'w':
            msg.linear.y = 1.0*velocity_multiplier;
            break;
        case 's':
            msg.linear.y = -1.0*velocity_multiplier;
            break;
        case 'z':
            msg.linear.z = 1.0*velocity_multiplier;
            break;
        case 'c':
            msg.linear.z = -1.0*velocity_multiplier;
            break;
        case 'k':
            msg.angular.z = 1.0*velocity_multiplier;
            break;
        case 'l':
            msg.angular.z = -1.0*velocity_multiplier;
            break;
        case 't':
            velocity_multiplier += 0.1;
            break;
        case 'b':
            velocity_multiplier -= 0.1;
            break;
	    case '.':
	        msg.linear.x = 0; msg.linear.y = 0; msg.linear.z = 0;
	        msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = 0;
	        break;
        default:
            ROS_INFO("Not a valid command! Proceeding with the latched twist");

    }
}
void killer(int signum){
    exit(0);
}
int main(int argc,char** argv){
    ros::init(argc,argv,"keyboard_teleop");
    ros::NodeHandle nh;
    ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    signal(SIGINT,killer);
    ros::ServiceClient enable_motors = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    hector_uav_msgs::EnableMotors srv;
    srv.request.enable = true;
    if(enable_motors.call(srv)){
        if(srv.response.success)
            ROS_INFO("Motors are enabled");
    }

    ROS_INFO("--------------_Hector QUadrotor Keyboard Teleop Module----------");
    ROS_INFO("\tIn order to manipulate quadrotor, use:\n\t\t Forward: w\n\t\t Reverse: s\n\t\t Left: a\n\t\t Right: d");
    ROS_INFO("\t\tEscalate: z \n\t\t Descend: c\n\t\t Turn left: k\n\t\t Turn right: l\n\t\t Throttle: t\n\t\t Break: b");

    ros::Rate rate(1.0);
    std::string command;
    while(ros::ok()){
        geometry_msgs::Twist vel_msg;
        hector_uav_msgs::ThrustCommand attitude_msg;
        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(0,&readSet);
        struct timeval tv = {1,0};
        select(1,&readSet, NULL,NULL,&tv);
        bool b = FD_ISSET(0,&readSet);
        if(b)
            std::cin>>command;
    
        ROS_INFO("command: %s",command.c_str());
        velocity_commander(command,vel_msg);
        pub_vel.publish(vel_msg);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}

