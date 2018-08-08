#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_quadrotor");
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::NodeHandle node_handle("~");

    const std::string PLANNING_GROUP = "DroneBody";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    for(auto d : move_group.getCurrentJointValues())
        ROS_INFO("Joint val: %lf",d);
    move_group.setWorkspace(-1000,-1000,-1000,1000,1000,1000);
    std::vector<double> multi_dof_joint_values = {10.44,0.18,7.302,0,0,0,1};
    move_group.setJointValueTarget(multi_dof_joint_values);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("Trial", "%s", success ? "SUCCESS" : "FAILED");
    if(success)
        move_group.execute(plan);
    return 0;
}