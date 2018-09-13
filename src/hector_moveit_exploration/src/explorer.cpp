#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/PoseAction.h>
#define XMIN -24.5
#define XMAX 24.5
#define YMIN -16
#define YMAX 16
#define ZMIN 0.2
#define ZMAX 10.0

class Quadrotor{
    private:
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> pose_client;
        std::unique_ptr<robot_state::RobotState> start_state;
        const double takeoff_altitude = 1.0;

        bool odom_received,trajectory_received;
        std::vector<double> odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;
        ros::Subscriber base_sub,plan_sub;
        ros::ServiceClient motor_enable_service; 

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
        {
            odometry_information[0] = msg->pose.pose.position.x;
            odometry_information[1] = msg->pose.pose.position.y;
            odometry_information[2] = msg->pose.pose.position.z;

            odometry_information[3] = msg->pose.pose.orientation.x;
            odometry_information[4] = msg->pose.pose.orientation.y;
            odometry_information[5] = msg->pose.pose.orientation.z;
            odometry_information[6] = msg->pose.pose.orientation.w;
            odom_received = true;
        }
        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
        {
            if(!odom_received) return;
            trajectory.clear();
            for(auto robot_traj: msg->trajectory){
                for(auto point : robot_traj.multi_dof_joint_trajectory.points){
                    geometry_msgs::Pose waypoint;
                    waypoint.position.x = point.transforms[0].translation.x;
                    waypoint.position.y = point.transforms[0].translation.y;
                    waypoint.position.z = point.transforms[0].translation.z;

                    waypoint.orientation.x = point.transforms[0].rotation.x;
                    waypoint.orientation.y = point.transforms[0].rotation.y;
                    waypoint.orientation.z = point.transforms[0].rotation.z;
                    waypoint.orientation.w = point.transforms[0].rotation.w;

                    trajectory.push_back(waypoint);
                }
            }
            trajectory_received = true;
        }
        void go(std::vector<double>& target){
            move_group->setJointValueTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            start_state->setVariablePositions(odometry_information);
            move_group->setStartState(*start_state);
            bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("Trial", "%s", success ? "SUCCESS" : "FAILED");
            while(!trajectory_received)
                ;
            hector_uav_msgs::PoseGoal goal;
            goal.target_pose.header.frame_id = "world";
            for(auto waypoint : trajectory){
                goal.target_pose.pose = waypoint;
                pose_client.sendGoal(goal);
                pose_client.waitForResult();
            }
            trajectory_received = false;
        }
    public:
        Quadrotor(const std::string PLANNING_GROUP,ros::NodeHandle& nh) : pose_client("/action/pose",true)
        {
            pose_client.waitForServer();
            odometry_information.resize(7);
            odom_received = false;
            trajectory_received = false;
            base_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&Quadrotor::poseCallback,this);
            plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Quadrotor::planCallback,this);

            move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();

            motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
            move_group->setPlannerId("RRTConnectkConfigDefault");
            move_group->setNumPlanningAttempts(10);
            move_group->setWorkspace(-50,-50,-50,50,50,50);
            
            start_state.reset(new robot_state::RobotState(move_group->getRobotModel()));
        }
        void takeoff()
        {
            hector_uav_msgs::EnableMotors srv;
            srv.request.enable = true;
            motor_enable_service.call(srv);
            while(!odom_received)
                ;
            std::vector<double> takeoff_pose = odometry_information;
            takeoff_pose[2] = takeoff_altitude;
            go(takeoff_pose);
            ROS_INFO("Takeoff successful");
        }
        void run()
        {
            while(ros::ok()){
                ros::spinOnce();
            }
            
        }
};



int main(int argc, char** argv)
{

    ros::init(argc, argv, "hector_explorer");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    const std::string PLANNING_GROUP = "DroneBody";
    Quadrotor quad(PLANNING_GROUP,std::ref(node_handle));
    quad.takeoff();
    quad.run();
    return 0;
}