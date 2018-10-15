#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_moveit_actions/ExecuteDroneTrajectoryAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>

#include <hector_uav_msgs/PoseAction.h>
#include <cmath>
#define _USE_MATH_DEFINES

#define MAX_SPEED 2.0
#define EPSILON 1e-4
class TrajectoryActionController{
    private:

        typedef actionlib::SimpleActionServer<hector_moveit_actions::ExecuteDroneTrajectoryAction> TrajectoryActionServer;
        typedef actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> OrientationClient;
        typedef hector_moveit_actions::ExecuteDroneTrajectoryResult Result;
        typedef hector_moveit_actions::ExecuteDroneTrajectoryFeedback Feedback;
        ros::NodeHandle nh_;
        ros::Publisher vel_pub;
        ros::Subscriber pose_sub;
        TrajectoryActionServer server_;
        OrientationClient orientation_client_;
        std::string action_name;

        geometry_msgs::Twist empty,cmd;
        std::vector<geometry_msgs::Pose> trajectory;
        geometry_msgs::Pose last_pose;
       
        Feedback feedback_;
        Result result_;
        bool success;
    public:
        TrajectoryActionController(std::string name) : action_name(name), 
            server_(nh_,name,boost::bind(&TrajectoryActionController::executeCB,this,_1),false),
            orientation_client_("/action/pose",true){
                orientation_client_.waitForServer();
                empty.linear.x = 0;empty.linear.y = 0; empty.linear.z = 0;
                empty.angular.x = 0;empty.angular.y = 0;empty.angular.z = 0;
                vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel",1);
                pose_sub = nh_.subscribe<nav_msgs::Odometry>("/ground_truth/state",10,&TrajectoryActionController::poseCallback,this);
                success = true;
                server_.start();
            
            }
        void executeCB(const hector_moveit_actions::ExecuteDroneTrajectoryGoalConstPtr &goal){
            
            trajectory = goal->trajectory;
            ROS_INFO_STREAM("Executing trajectory!");
            for(int k=0; k<trajectory.size(); k++){
                if(server_.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("Preempt requested");
                    this->success = false;
                    break;
                }
               
                geometry_msgs::Pose p = trajectory[k];
                geometry_msgs::Pose waypoint = p;
               
                   
                double y_diff = waypoint.position.y - last_pose.position.y;
                double x_diff = waypoint.position.x - last_pose.position.x;
                double yaw = atan2(y_diff,x_diff);
                    
                double required_yaw = limitAngleRange(yaw+M_PI);
                tf::Quaternion q;
                tf:quaternionMsgToTF(last_pose.orientation, q);
                double tmp, current_yaw;
                tf::Matrix3x3(q).getRPY(tmp,tmp, current_yaw);
                required_yaw /=2;
                if(fabs(y_diff)>EPSILON && fabs(x_diff)>EPSILON && fabs(required_yaw - current_yaw)>0.01){ //Prevent 0 division
                    tf::Quaternion q = tf::createQuaternionFromYaw(yaw+M_PI);
                    waypoint.orientation.x = q.x();
                    waypoint.orientation.y = q.y();
                    waypoint.orientation.z = q.z();
                    waypoint.orientation.w = q.w();

                    waypoint.position = last_pose.position;
                    hector_uav_msgs::PoseGoal orientation_goal;
                    orientation_goal.target_pose.pose = waypoint;
                    orientation_goal.target_pose.header.frame_id="world";
                    orientation_client_.sendGoal(orientation_goal,actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleDoneCallback(),
                        actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleActiveCallback(),
                        actionlib::SimpleActionClient<hector_uav_msgs::PoseAction>::SimpleFeedbackCallback());
                    //orientation_client_.waitForResult();
                    ros::Duration(0.05).sleep();
                }
                
                
                waypoint.position = p.position;
        
                publishTranslationComand(waypoint);
                vel_pub.publish(empty);
                last_pose.position=waypoint.position;
                last_pose.orientation=waypoint.orientation;
                feedback_.current_pose = last_pose;
                server_.publishFeedback(feedback_);
                
            }
            if(!this->success){
                result_.result_code = Result::COLLISION_IN_FRONT;
                server_.setPreempted(result_);
                return ;
            }
            ROS_INFO_STREAM("Executed trajectory!");
            result_.result_code = Result::SUCCESSFUL;
            server_.setSucceeded(result_);
            
        }
        inline double limitAngleRange(double angle){
            while(angle>M_PI)
                angle -= 2*M_PI;
            while(angle<=-M_PI)
                angle += 2*M_PI;
            return angle;
        }
        inline void limitVelocity(){
            double mag = sqrt(pow(cmd.linear.x,2) + pow(cmd.linear.y,2) + pow(cmd.linear.z,2));
            if(mag>MAX_SPEED){
                cmd.linear.x /= mag/MAX_SPEED;
                cmd.linear.y /= mag/MAX_SPEED;
                cmd.linear.z /= mag/MAX_SPEED; 
            }
        }

        bool publishTranslationComand(geometry_msgs::Pose& p){
            float Kc_linear = 2.0;
            float Kc_bearing = 1.0;
            
            tf::Vector3 pos;
            pos.setX(p.position.x-last_pose.position.x);
            pos.setY(p.position.y-last_pose.position.y);
            pos.setZ(p.position.z-last_pose.position.z);

            cmd.linear.x=pos.getX()*Kc_linear;
            cmd.linear.y=0;
            cmd.linear.z=pos.getZ()*Kc_linear;
            cmd.angular.x=cmd.angular.y=0,cmd.angular.z=0;
        
                
            limitVelocity();
            vel_pub.publish(cmd);

            ros::spinOnce();
            ros::Duration(0.5).sleep();
            return true;
        }

        void poseCallback(const nav_msgs::Odometry::ConstPtr & msg)
        {
            last_pose = msg->pose.pose;
        }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_executor");
  TrajectoryActionController controller("/action/trajectory");
  ros::spin();
  return 0;
}