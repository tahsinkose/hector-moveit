#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <nav_msgs/Odometry.h>
class QuadrotorPlanner{
    private:
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        std::unique_ptr<robot_state::RobotState> start_state;
        bool odom_received;
        std::vector<double> odometry_information;
        ros::Subscriber base_sub;
        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg){
            odometry_information[0] = msg->pose.pose.position.x;
            odometry_information[1] = msg->pose.pose.position.y;
            odometry_information[2] = msg->pose.pose.position.z;

            odometry_information[3] = msg->pose.pose.orientation.x;
            odometry_information[4] = msg->pose.pose.orientation.y;
            odometry_information[5] = msg->pose.pose.orientation.z;
            odometry_information[6] = msg->pose.pose.orientation.w;
            odom_received = true;
        }
    public:
        QuadrotorPlanner(const std::string PLANNING_GROUP,ros::NodeHandle& nh){
            odometry_information.resize(7);
            base_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&QuadrotorPlanner::poseCallback,this);
            move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();

            move_group->setPlannerId("RRTConnectkConfigDefault");
            move_group->setNumPlanningAttempts(10);
            move_group->setWorkspace(-50,-50,-50,50,50,50);
            
            start_state.reset(new robot_state::RobotState(move_group->getRobotModel()));
        }
        void run(){

            std::vector<double> test_joint_values = {0,0,10,0,0,0,1};
            move_group->setJointValueTarget(test_joint_values);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            while(!odom_received)
                ;
            start_state->setVariablePositions(odometry_information);
            move_group->setStartState(*start_state);
            bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("Trial", "%s", success ? "SUCCESS" : "FAILED");
        }
};



int main(int argc, char** argv)
{

    ros::init(argc, argv, "move_quadrotor");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    const std::string PLANNING_GROUP = "DroneBody";
    QuadrotorPlanner planner(PLANNING_GROUP,std::ref(node_handle));
    planner.run();
    return 0;
}