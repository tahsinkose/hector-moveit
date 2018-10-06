#include <Explorer.h>

Quadrotor::Quadrotor(ros::NodeHandle& nh) : trajectory_client("/action/trajectory",true)
{
    trajectory_client.waitForServer();
    odom_received = false;
    trajectory_received = false;
    collision = false;
    std::vector<double> xlimits = {XMIN,XMAX};
    std::vector<double> ylimits = {YMIN,YMAX};
    std::vector<double> zlimits = {ZMIN,ZMAX};
    
    /*for(int i=0;i<2;i++)
        for(int j=0;j<2;j++)
            for(int k=0;k<2;k++){
                double x_offset = (i==0 ? 1.0 : -1.0);
                double y_offset = (j==0 ? 1.0 : -1.0);
                double z_offset = (k==0 ? 1.5 : -1.0);
                geometry_msgs::Pose corner;
                corner.position.x = xlimits[i] + x_offset;
                corner.position.y = ylimits[j] + y_offset;
                corner.position.z = zlimits[k] + z_offset;
                corner.orientation.w = 1;

                frontiers.push_back(corner);
            }
    */
    base_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&Quadrotor::poseCallback,this);
    plan_sub = nh.subscribe<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path",1,&Quadrotor::planCallback,this);

    move_group.reset(new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kmodel = robot_model_loader.getModel();
    
    motor_enable_service = nh.serviceClient<hector_uav_msgs::EnableMotors>("/enable_motors");
    planning_scene_service = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

    move_group->setPlannerId("RRTConnectkConfigDefault");
    move_group->setNumPlanningAttempts(10);
    move_group->setWorkspace(XMIN,YMIN,ZMIN,XMAX,YMAX,ZMAX);
    
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
        octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
        octomap::OcTree* current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);
        
        double resolution = current_map->getResolution();
        int unknown = 0, known = 0;
        for(double ix=XMIN;ix<XMAX;ix+=resolution){
            for(double iy=YMIN;iy<YMAX;iy+=resolution){
                for(double iz=ZMIN;iz<ZMAX;iz+=resolution){
                    if(!current_map->search(ix,iy,iz))
                        unknown++;
                    else
                        known++;
                }
            }
        }
        ROS_INFO("Coverage of Orchard Volume: %lf Percent",known*100.0/(float)(unknown+known));
        
        delete current_map;
        std::vector<size_t> invalid_indices;
        this->isPathValid = this->planning_scene->isPathValid(plan_start_state,plan_trajectory,PLANNING_GROUP,true,&invalid_indices);
        ros::spinOnce();
        bool too_close = false;
        for(int i=0;i<invalid_indices.size();i++){
            for(int j=0;j<plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms.size();j++){
                
                double x = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.x;
                double y = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.y;
                double z = plan_trajectory.multi_dof_joint_trajectory.points[invalid_indices[i]].transforms[j].translation.z;

                double dist = sqrt(pow(x-odometry_information.position.x,2) + pow(y-odometry_information.position.y,2) + pow(z-odometry_information.position.z,2));
                if(dist < 0.5) too_close = true;
            }
        }
        
       if(!isPathValid && !too_close){
            //TODO: this->move_group->stop(); When migrating to complete MoveIt! ExecuteService, this will work as expected.
            this->trajectory_client.cancelGoal();
            this->collision = true;
            ROS_INFO("Trajectory is now in collision with the world");
        }
    }
    else
        ROS_INFO("Couldn't fetch the planning scene");
}

double Quadrotor::countFreeVolume(const octomap::OcTree *octree) {
    double resolution = octree->getResolution();
    int unknown = 0, known = 0;;
    for(double ix=XMIN;ix<XMAX;ix+=resolution){
        for(double iy=YMIN;iy<YMAX;iy+=resolution){
            for(double iz=ZMIN;iz<ZMAX;iz+=resolution){
                if(!octree->search(ix,iy,iz))
                    unknown++;
                else
                    known++;
            }
        }
    }
    return known*100.0/(float)(unknown+known);
}

double Quadrotor::calc_MI(const octomap::OcTree *octree, const geometry_msgs::Point& point, const octomap::Pointcloud &hits, const double before) {
    auto octree_copy = new octomap::OcTree(*octree);
    octomap::point3d orig(point.x,point.y,point.z);
    octree_copy->insertPointCloud(hits, orig, 100, true, true);
    double after = countFreeVolume(octree_copy);
    delete octree_copy;
    return after - before;
}

octomap::Pointcloud Quadrotor::castSensorRays(const octomap::OcTree* curr_tree,const geometry_msgs::Pose& pose){
    octomap::Pointcloud hits;
    double height = 480;
    double width = 640;
    double horizontal_fov = 90 * M_PI / 180;
    double vertical_fov = 60 * M_PI / 180;
    double max_range = 100;

    double angle_inc_ver = vertical_fov / height;
    double angle_inc_hor = horizontal_fov / width;
    octomap::Pointcloud sensorRays;
    for(double j=-height/2;j<height/2;j+=15){
        for(double i=-width/2;i<width/2;i+=15){
            octomap::point3d vec(1,0,0);
            vec.rotate_IP(0.0, j * angle_inc_ver, i * angle_inc_hor);
            sensorRays.push_back(vec);
        }
    }
    tf::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w);
    tf::Matrix3x3 m(q);
    double tmp, yaw;
    m.getRPY(tmp,tmp, yaw);
    octomap::Pointcloud RaysToCast;
    RaysToCast.push_back(sensorRays);
    RaysToCast.rotate(0,0,yaw);
    octomap::point3d end;
    octomap::point3d position(pose.position.x,pose.position.y,pose.position.z);
    for(int i = 0; i < RaysToCast.size(); i++) {
        if(curr_tree->castRay(position, RaysToCast.getPoint(i), end, true, max_range)) {
            hits.push_back(end);
        } else {
            end = RaysToCast.getPoint(i) * max_range;
            end += position;
            hits.push_back(end);
        }
    }
    return hits;
}

void Quadrotor::findFrontier()
{
    ROS_INFO("Looking for frontiers");
    moveit_msgs::GetPlanningScene srv;
    srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
    ros::spinOnce();
    while(!frontiers.empty())
        frontiers.pop();
    if(planning_scene_service.call(srv)){
        this->planning_scene->setPlanningSceneDiffMsg(srv.response.scene);
        octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
        octomap::OcTree* current_map = (octomap::OcTree*)octomap_msgs::msgToMap(octomap);
        
        double resolution = current_map->getResolution();
        
        std::vector<std::pair<double, geometry_msgs::Pose> > candidate_frontiers;
        for(octomap::OcTree::leaf_iterator n = current_map->begin_leafs(current_map->getTreeDepth()); n != current_map->end_leafs(); ++n)
        {
            //if(candidate_frontiers.size()==15) break;
            if(!current_map->isNodeOccupied(*n))
            {
                double x_cur = n.getX();
                double y_cur = n.getY();
                double z_cur = n.getZ();
                
                bool frontier = false;
                bool already_explored = false;
                for(auto a : explored){
                    if(fabs(x_cur - a.position.x) < 2.0 && fabs(y_cur - a.position.y) < 2.0 && fabs(z_cur - a.position.z) < 2.0){
                        already_explored = true;
                        break;
                    }
                }
                if(already_explored)
                    continue;
                if(x_cur < XMIN + resolution || x_cur > XMAX - resolution
                || y_cur < YMIN + resolution || y_cur > YMAX - resolution
                || z_cur < ZMIN + resolution || z_cur > ZMAX - resolution) continue;
                for (double x_cur_buf = x_cur - resolution; x_cur_buf < x_cur + resolution; x_cur_buf += resolution)
                {   
                    for (double y_cur_buf = y_cur - resolution; y_cur_buf < y_cur + resolution; y_cur_buf += resolution)
                    {
                        
                        octomap::OcTreeNode *n_cur_frontier = current_map->search(x_cur_buf, y_cur_buf, z_cur);
                        if(!n_cur_frontier)
                        {
                            frontier = true;
                        }
                    }
                }
                if(frontier){
                    geometry_msgs::Pose p;
                    p.position.x = x_cur;p.position.y = y_cur;p.position.z = z_cur;
                    p.orientation.w = 1;
                    double dist = sqrt(pow(p.position.x - odometry_information.position.x,2) + pow(p.position.y - odometry_information.position.y,2) + pow(p.position.z - odometry_information.position.z,2));
                    if(dist > 2)
                        candidate_frontiers.push_back({dist,p});
                }
            }
        }
        std::sort(candidate_frontiers.begin(),candidate_frontiers.end(), 
            [](const DistancedPoint& x, const DistancedPoint& y){
                return x.first > y.first;
            });
        std::vector<int> indices(candidate_frontiers.size());
        if(candidate_frontiers.size() > 10){
            
            std::iota(indices.begin(),indices.end(),0);
            std::random_shuffle(indices.begin(),indices.end());
            indices.erase(indices.begin()+10,indices.end()); 
        }
        for(int i=0;i<indices.size();i++)
            frontiers.push(candidate_frontiers[i]);
        /*double before = countFreeVolume(current_map);
        std::vector<DistancedPoint > vec;
        
        omp_set_num_threads(4);
        auto start = get_time::now();
        #pragma omp parallel
        {
            #pragma omp single
            {
               ROS_INFO("Num threads: %d",omp_get_num_threads());
            }
            std::vector<DistancedPoint > vec_private;
            #pragma omp for nowait schedule(static)
            for(int i=0;i<indices.size();i++)
            {
                geometry_msgs::Pose frontier = candidate_frontiers[indices[i]].second;
                octomap::Pointcloud hits = castSensorRays(current_map,frontier);
                double MI = calc_MI(current_map,frontier.position,hits,before);
                //ROS_INFO("Mutual information of candidate frontier [%lf,%lf,%lf] = %lf",frontier.position.x,frontier.position.y,frontier.position.z,MI);
                vec_private.push_back({MI,frontier});
                
            }
            
            #pragma omp for schedule(static) ordered
            for(int i=0; i<omp_get_num_threads(); i++) {
                #pragma omp ordered
                vec.insert(vec.end(), vec_private.begin(), vec_private.end());
            }
        }
        
        auto end = get_time::now();
	    auto differ = end - start;
	    ROS_INFO("Elapsed time is :  %lu ns",chrono::duration_cast<ns>(differ).count());
        for(auto d : vec)
            frontiers.push(d);
        */
    }
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
    
    this->collision = false;
    ROS_INFO("Try to start from [%lf,%lf,%lf]",odometry_information.position.x,odometry_information.position.y,odometry_information.position.z);
    ROS_INFO("Try to go to [%lf,%lf,%lf]",target_.position.x,target_.position.y,target_.position.z);
    this->start_state->setVariablePositions(start_state_);
    this->move_group->setStartState(*start_state);

    this->isPathValid = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if(this->isPathValid){
        
        this->plan_start_state = plan.start_state_;
        this->plan_trajectory = plan.trajectory_;
        while(!trajectory_received){
            ROS_INFO("Waiting for trajectory");
            ros::Duration(0.2).sleep();
        }
        hector_moveit_actions::ExecuteDroneTrajectoryGoal goal;
        
        for(int i=0;i<trajectory.size();i++){
            if(i==0){
                double y_diff = trajectory[i].position.y - odometry_information.position.y;
                double x_diff = trajectory[i].position.x - odometry_information.position.x;
                double yaw = atan2(y_diff,x_diff);
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i].orientation.x = q.x();
                    trajectory[i].orientation.y = q.y();
                    trajectory[i].orientation.z = q.z();
                    trajectory[i].orientation.w = q.w();
                }
            }
            else if(i+1<trajectory.size()){
                geometry_msgs::Pose next_waypoint = trajectory[i+1];
                double y_diff = next_waypoint.position.y - trajectory[i].position.y;
                double x_diff = next_waypoint.position.x - trajectory[i].position.x;
                double yaw = atan2(y_diff,x_diff);
                
                if(fabs(y_diff)>EPSILON || fabs(x_diff)>EPSILON ){ //Prevent 0 division
                    
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    trajectory[i+1].orientation.x = q.x();
                    trajectory[i+1].orientation.y = q.y();
                    trajectory[i+1].orientation.z = q.z();
                    trajectory[i+1].orientation.w = q.w();
                }
            }
            goal.trajectory.push_back(trajectory[i]);
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
            rate.sleep();
        bool success = false;
        findFrontier();
        
        do{
            if(frontiers.empty()) break;
            geometry_msgs::Pose _goal = frontiers.front().second;
            frontiers.pop();
            explored.push_back(_goal); // Valid or not, make sure that will not be offered as candidate again.
            bool invalid = false;
            for(int i=0;i<invalid_poses.size();i++){
                
                if(sqrt(pow(invalid_poses[i].position.x - _goal.position.x,2) + pow(invalid_poses[i].position.y - _goal.position.y,2) 
                    + pow(invalid_poses[i].position.z - _goal.position.z,2)) < 1.5){
                    invalid = true;
                    invalid_poses.push_back(_goal);
                    break;
                }
            }
            if(invalid) continue;

            success = go(_goal);
            if(!success) invalid_poses.push_back(_goal);
            ros::spinOnce();
            rate.sleep();
        }while(!success);
    }
}