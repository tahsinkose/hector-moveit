#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <nav_msgs/Odometry.h>
std::vector<double> odom(7);
bool odom_received = false;

void poseCallback(const nav_msgs::Odometry::ConstPtr & msg){
    odom[0] = msg->pose.pose.position.x;
    odom[1] = msg->pose.pose.position.y;
    odom[2] = msg->pose.pose.position.z;

    odom[3] = msg->pose.pose.orientation.x;
    odom[4] = msg->pose.pose.orientation.y;
    odom[5] = msg->pose.pose.orientation.z;
    odom[6] = msg->pose.pose.orientation.w;
    odom_received = true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "move_quadrotor");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    ros::Subscriber base_sub = node_handle.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&poseCallback);
    const std::string PLANNING_GROUP = "DroneBody";
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();
    
    std::vector<double> test_joint_values = {0,0,10,0,0,0,1};
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setNumPlanningAttempts(10);
    move_group.setWorkspace(-50,-50,-50,50,50,50);
    
    move_group.setJointValueTarget(test_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    robot_state::RobotState start_state(move_group.getRobotModel());

    while(!odom_received)
        ;
    start_state.setVariablePositions(odom);
    move_group.setStartState(start_state);
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Trial", "%s", success ? "SUCCESS" : "FAILED");
    return 0;
}