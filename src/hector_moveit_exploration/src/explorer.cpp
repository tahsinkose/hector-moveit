#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/conversions.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPlanningScene.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <hector_uav_msgs/EnableMotors.h>
#include <hector_uav_msgs/PoseAction.h>

#include <octomap/OcTree.h>

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
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        const double takeoff_altitude = 1.0;

        bool odom_received,trajectory_received;
        bool isPathValid;
        geometry_msgs::Pose odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;
        std::vector<geometry_msgs::Pose> orchard_points;
        ros::Subscriber base_sub,plan_sub;
        ros::ServiceClient motor_enable_service; 
        ros::ServiceClient planning_scene_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;

        const std::string PLANNING_GROUP = "DroneBody";

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
        {
            odometry_information = msg->pose.pose;
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

        void collisionCallback(){
            moveit_msgs::GetPlanningScene srv;
            srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
            
            if(planning_scene_service.call(srv)){
                this->planning_scene->setPlanningSceneDiffMsg(srv.response.scene);
                /*octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
                octomap::OcTree* current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);
                current_map->writeBinary("/home/tahsincan/tree.bt");
                ROS_INFO("Saving the tree");*/
                std::vector<size_t> invalid_indices;
                this->isPathValid = this->planning_scene->isPathValid(plan_start_state,plan_trajectory,PLANNING_GROUP,true,&invalid_indices);
                
                if(!isPathValid){
                    //TODO: this->move_group->stop(); When migrating to complete MoveIt! ExecuteService, this will work as expected.
                    this->pose_client.cancelGoal();
                    ROS_INFO("Trajectory is now in collision with the world");
                }
            }
            else
                ROS_INFO("Couldn't fetch the planning scene");
        }

        bool go(geometry_msgs::Pose& target_){
            
            std::vector<double> target(7);
            target[0] = target_.position.x;
            target[1] = target_.position.y;
            target[2] = target_.position.z;
            target[3] = target_.orientation.x;
            target[4] = target_.orientation.y;
            target[5] = target_.orientation.z;
            target[6] = target_.orientation.w;
            
            std::vector<double> start_state_(7);
            start_state_[0] = odometry_information.position.x;
            start_state_[1] = odometry_information.position.y;
            start_state_[2] = odometry_information.position.z;
            start_state_[3] = odometry_information.orientation.x;
            start_state_[4] = odometry_information.orientation.y;
            start_state_[5] = odometry_information.orientation.z;
            start_state_[6] = odometry_information.orientation.w;
            
            this->move_group->setJointValueTarget(target);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            
            ROS_INFO("Try to start from [%lf,%lf,%lf]",odometry_information.position.x,odometry_information.position.y,odometry_information.position.z);
            ROS_INFO("Try to go to [%lf,%lf,%lf]",target_.position.x,target_.position.y,target_.position.z);
            this->start_state->setVariablePositions(start_state_);
            this->move_group->setStartState(*start_state);

            this->isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if(this->isPathValid){
                
                this->plan_start_state = plan.start_state_;
                this->plan_trajectory = plan.trajectory_;
                while(!trajectory_received)
                    ;
                hector_uav_msgs::PoseGoal goal;
                goal.target_pose.header.frame_id = "world";
                for(auto waypoint : trajectory){
                    if(!this->isPathValid) break;
                    goal.target_pose.pose = waypoint;
                    ROS_INFO("Next orientation [%lf,%lf,%lf,%lf]",waypoint.orientation.x,waypoint.orientation.y,waypoint.orientation.z,waypoint.orientation.w);
                    pose_client.sendGoal(goal,actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleDoneCallback(),
                                        boost::bind(&Quadrotor::collisionCallback,this),
                                        actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleFeedbackCallback());
                    pose_client.waitForResult();
                }
                this->trajectory_received = false;
                this->odom_received = false;
            }
            return this->isPathValid;
        }
    public:
        Quadrotor(ros::NodeHandle& nh) : pose_client("/action/pose",true)
        {
            pose_client.waitForServer();
            odom_received = false;
            trajectory_received = false;
            std::vector<double> xlimits = {XMIN,XMAX};
            std::vector<double> ylimits = {YMIN,YMAX};
            std::vector<double> zlimits = {ZMIN,ZMAX};
            for(int i=0;i<2;i++)
                for(int j=0;j<2;j++)
                    for(int k=0;k<2;k++){
                        double x_offset = (i==0 ? 0.5 : -0.5);
                        double y_offset = (j==0 ? 0.5 : -0.5);
                        double z_offset = (k==0 ? 0.5 : -0.5);
                        geometry_msgs::Pose corner;
                        corner.position.x = xlimits[i] + x_offset;
                        corner.position.y = ylimits[j] + y_offset;
                        corner.position.z = zlimits[k] + z_offset;
                        corner.orientation.w = 1;

                        orchard_points.push_back(corner);
                    }

            base_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&Quadrotor::poseCallback,this);
            plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Quadrotor::planCallback,this);

            move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
            robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
            robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();
            
            motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
            planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

            move_group->setPlannerId("RRTConnectkConfigDefault");
            move_group->setNumPlanningAttempts(10);
            move_group->setWorkspace(-50,-50,0,50,50,ZMAX);
            
            start_state.reset(new robot_state::RobotState(move_group->getRobotModel()));
            planning_scene.reset(new planning_scene::PlanningScene(kmodel));
            
        }
        void takeoff()
        {
            hector_uav_msgs::EnableMotors srv;
            srv.request.enable = true;
            motor_enable_service.call(srv);
            while(!odom_received)
                ;
            geometry_msgs::Pose takeoff_pose = odometry_information;
            takeoff_pose.position.z = takeoff_altitude;
            go(takeoff_pose);
            ROS_INFO("Takeoff successful");
        }
        void run()
        {
            ros::Rate rate(2);
            while(ros::ok()){
                while(!odom_received)
                    ;
                for(int i=0;i<orchard_points.size();i++){
                    while(!go(orchard_points[i])){ //Send until reached to the specified point.
                        ros::spinOnce();
                        rate.sleep();
                    }
                }
            }
            
        }
};



int main(int argc, char** argv)
{

    ros::init(argc, argv, "hector_explorer");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle("~");
    
    Quadrotor quad(std::ref(node_handle));
    quad.takeoff();
    quad.run();
    return 0;
}