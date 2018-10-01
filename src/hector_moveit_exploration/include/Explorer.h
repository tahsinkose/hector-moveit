#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <tf/tf.h>

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
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>

#include <octomap/OcTree.h>


#define _USE_MATH_DEFINES
#include <cmath>
#include <queue>
#define XMIN -24.5
#define XMAX 24.5
#define YMIN -16
#define YMAX 16
#define ZMIN 0.2
#define ZMAX 4.0

#define EPSILON 1e-4

typedef std::pair<double,geometry_msgs::Pose> DistancedPoint;

class Compare{
    public: 
        bool operator()(DistancedPoint& lhs, DistancedPoint & rhs)
        {
            return lhs.first < rhs.first;
        }
};

class Quadrotor{
    private:
        std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
        actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction> trajectory_client;
        std::unique_ptr<robot_state::RobotState> start_state;
        std::unique_ptr<planning_scene::PlanningScene> planning_scene;
        const double takeoff_altitude = 1.0;

        bool odom_received,trajectory_received;
        bool isPathValid;
        bool collision;

        geometry_msgs::Pose odometry_information;
        std::vector<geometry_msgs::Pose> trajectory;
        std::priority_queue<DistancedPoint,std::vector<DistancedPoint>, Compare> orchard_points;
        std::vector<geometry_msgs::Pose> explored;

        ros::Subscriber base_sub,plan_sub;
        ros::ServiceClient motor_enable_service; 
        ros::ServiceClient planning_scene_service;

        moveit_msgs::RobotState plan_start_state;
        moveit_msgs::RobotTrajectory plan_trajectory;
    
        const std::string PLANNING_GROUP = "DroneBody";

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg);

        void planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg);

        void collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback);
        
        void findFrontier(const actionlib::SimpleClientGoalState& state,const hector_moveit_actions::ExecuteDroneTrajectoryResultConstPtr& result);

        bool go(geometry_msgs::Pose& target_);
    
    public:
        Quadrotor(ros::NodeHandle& nh);
        void takeoff();
        void run();
        
};