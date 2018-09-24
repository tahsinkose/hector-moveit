#include <Explorer.h>

Quadrotor::Quadrotor(ros::NodeHandle& nh) : trajectory_client("/action/trajectory",true)
{
    trajectory_client.waitForServer();
    odom_received = false;
    trajectory_received = false;
    std::vector<double> xlimits = {XMIN,XMAX};
    std::vector<double> ylimits = {YMIN,YMAX};
    std::vector<double> zlimits = {ZMIN,ZMAX};
    
    for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
            for(int k=0;k<2;k++){
                double x_offset = (i==0 ? 1.0 : -1.0);
                double y_offset = (j==0 ? 1.0 : -1.0);
                double z_offset = (k==0 ? 1.0 : -1.0);
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

void Quadrotor::poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    odometry_information = msg->pose.pose;
    odom_received = true;
}

void Quadrotor::planCallback(const moveit_msgs::DisplayTrajectory::ConstPtr& msg)
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

void Quadrotor::collisionCallback(const hector_moveit_actions::ExecuteDroneTrajectoryFeedbackConstPtr& feedback)
{
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
            this->trajectory_client.cancelGoal();
            ROS_INFO("Trajectory is now in collision with the world");
        }
    }
    else
        ROS_INFO("Couldn't fetch the planning scene");
}

bool Quadrotor::go(geometry_msgs::Pose& target_)
{
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
    moveit_msgs::OrientationConstraint ocm;
    ocm.link_name = "camera_link";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;
    moveit_msgs::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group->setPathConstraints(constraints);
    this->isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(this->isPathValid){
        
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received)
            ;
        hector_moveit_actions::ExecuteDroneTrajectoryGoal goal;
        
        for(int i=0;i<trajectory.size();i++){
            
            if(i+1<trajectory.size()){
                geometry_msgs::Pose next_waypoint = trajectory[i+1];
                double y_diff = next_waypoint.position.y - trajectory[i].position.y;
                double x_diff = next_waypoint.position.x - trajectory[i].position.x;
                double yaw = atan2(y_diff,x_diff);
                
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i].orientation.x = q.x();
                    trajectory[i].orientation.y = q.y();
                    trajectory[i].orientation.z = q.z();
                    trajectory[i].orientation.w = q.w();
                }
                goal.trajectory.push_back(trajectory[i]);
            }
        }    
        ROS_INFO("Send Trajectory Goal");
        trajectory_client.sendGoal(goal,actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleDoneCallback(),
                            actionlib::SimpleActionClient<hector_moveit_actions::ExecuteDroneTrajectoryAction>::SimpleActiveCallback(),
                            boost::bind(&Quadrotor::collisionCallback,this,_1));
        trajectory_client.waitForResult();
        ROS_INFO("Trajectory is traversed");
        this->trajectory_received = false;
        this->odom_received = false;
    }
    return this->isPathValid;
}

void Quadrotor::takeoff()
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

void Quadrotor::run()
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