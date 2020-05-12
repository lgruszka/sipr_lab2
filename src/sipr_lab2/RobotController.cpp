
#include "sipr_lab2/RobotController.h"

#include <cmath>

#include <angles/angles.h>


double sgn(double v)
{
    if(v>0.0)
    {
        return 1.0;
    }
    else if(v<0.0)
    {
        return -1.0;
    }
    
    return 0.0;
}

RobotController::RobotController(Config const& cfg) : nh_("~"), cfg_(cfg) 
{
    joint_state_msg_.header.frame_id = "base_link";
    joint_state_msg_.name.resize(2);
    joint_state_msg_.name[0] = "base_left_wheel_joint";
    joint_state_msg_.name[1] = "base_right_wheel_joint";

    joint_state_msg_.position.resize(2);
    joint_state_msg_.position[0] = 0;
    joint_state_msg_.position[1] = 0;

    joint_state_msg_.velocity.resize(2);
    joint_state_msg_.velocity[0] = 0;
    joint_state_msg_.velocity[1] = 0;

    cfg_.r = 0.09;
    cfg_.b = 0.316;

//    cfg_.k1 = ?;
//    cfg_.k2 = ?;
//    cfg_.k3 = ?;
//
//    cfg_.ksx = ?;
//    cfg_.s_period = ?;

    computePathTraversalTime();
    
    controller_state_ = ControllerState::Ready;
    ROS_INFO("Robot controller state: %d", int(controller_state_));
    
    sub_goal_pose_ = nh_.subscribe(cfg_.robot_namespace+"/goal", 1,
            &RobotController::goalPoseCallback, this);
    
    sub_laser_scan_ = nh_.subscribe(cfg_.robot_namespace+"/base_scan", 1,
            &RobotController::laserScanCallback, this);

    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(cfg_.robot_namespace+"/cmd_vel", 1);
    pub_path_ = nh_.advertise<nav_msgs::Path>(cfg_.robot_namespace+"/path", 1);
    pub_next_pose_ = nh_.advertise<geometry_msgs::PoseStamped>(cfg_.robot_namespace+"/next_pose", 1);
    pub_joint_state_ = nh_.advertise<sensor_msgs::JointState>(cfg_.robot_namespace+"/joint_states", 1);
}

RobotController::~RobotController() 
{
    
}

geometry_msgs::Pose2D RobotController::getRobotGlobalPose(ros::Time time_now)
{
    geometry_msgs::Pose2D pose;
    
    //Pobranie transformacji między układem globalnym a układem robota
    tf::StampedTransform transform_box_robot;
    try 
    {
        ros::Duration timeout(cfg_.time_step/2.0);
        
        tf_.waitForTransform(cfg_.globale_frame_id, cfg_.robot_frame_id,
                time_now, timeout);
        
        tf_.lookupTransform(cfg_.globale_frame_id, cfg_.robot_frame_id,
                time_now, transform_box_robot);
        
        pose.x = transform_box_robot.getOrigin().getX();
        pose.y = transform_box_robot.getOrigin().getY();
        pose.theta = tf::getYaw(transform_box_robot.getRotation());
        
    } catch (tf::TransformException& ex) 
    {
        ROS_WARN("TF exception:\n%s", ex.what());
    }
    
    return pose;
}

void RobotController::laserScanCallback(sensor_msgs::LaserScan const& laser_msg)
{ 
    laser_msg_ = laser_msg;
}

void RobotController::goalPoseCallback(geometry_msgs::PoseStamped const& goal_pose_msg)
{
    goal_pose_.x = goal_pose_msg.pose.position.x;
    goal_pose_.y = goal_pose_msg.pose.position.y;
    goal_pose_.theta = tf::getYaw(goal_pose_msg.pose.orientation);
    
    publishPathMsg();
    
    controller_state_ = ControllerState::PathTracking;
    path_tracking_start_time = ros::Time::now();
    ROS_INFO("Robot controller state: %d", int(controller_state_));
}

void RobotController::computePathTraversalTime()
{
    double distance = 0.0;
    int steps = 1000;
    
    geometry_msgs::Point p1 = getPathPoint(0.0);
    
    for(int i=1; i<=steps; i++)
    {
        geometry_msgs::Point p2 = getPathPoint( double(i)/steps );
        
        distance += distanceXY(p1, p2);
        
        p1 = p2;
    }
    
    path_traversal_time = distance / cfg_.path_tracking_velocity;
    
    ROS_INFO("Estimated path traversal time: %f", path_traversal_time);
}

void RobotController::publishPathMsg()
{
    nav_msgs::Path path;
    path.header.frame_id = cfg_.globale_frame_id;
    path.header.stamp = path_tracking_start_time;
    
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = cfg_.globale_frame_id;
    
    for(double time = 0.0; time < path_traversal_time; time+=cfg_.time_step)
    {        
        pose.header.stamp = path_tracking_start_time + ros::Duration(time);
        geometry_msgs::Pose2D pose_2d = getPathPose(pose.header.stamp);
        pose.pose.position.x = pose_2d.x;
        pose.pose.position.y = pose_2d.y;
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_2d.theta);
        
        path.poses.push_back(pose);
    }
    
    pub_path_.publish(path);
}


void RobotController::publishNextPoseMsg(geometry_msgs::Pose2D pose_2d)
{
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = cfg_.globale_frame_id;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = pose_2d.x;
    pose.pose.position.y = pose_2d.y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose_2d.theta);
    
    pub_next_pose_.publish(pose);
}


double RobotController::distanceXY(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double RobotController::distanceXY(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double RobotController::distanceTheta(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2)
{    
    double angular_distance = angles::shortest_angular_distance(p1.theta, p2.theta); // najkrótsza odległość między kątami < -PI, PI >
    
    return angular_distance;
}

geometry_msgs::Point RobotController::getPathPoint(double t)
{
    geometry_msgs::Point point;
    
    point.x = cfg_.path_scale * std::sin(t * 4.0 * M_PI);
    point.y = cfg_.path_scale * (std::sin(t / 2.0 * 4.0 * M_PI) + std::cos(2.0 * t * 4.0 * M_PI)/7.0);
    
    return point;    
}

geometry_msgs::Point RobotController::getPathPoint(ros::Time time)
{
    ros::Duration d = (time - path_tracking_start_time);
    double t = d.toSec() / path_traversal_time;
        
    geometry_msgs::Point point = getPathPoint(t);
        
    return point;    
}

geometry_msgs::Pose2D RobotController::getPathPose(ros::Time time_now)
{
    geometry_msgs::Point p1 = getPathPoint(time_now);
    geometry_msgs::Point p2 = getPathPoint(time_now + ros::Duration(cfg_.time_step));
    
    tf::Transform pose_transform;
    pose_transform.setOrigin(tf::Vector3(p1.x, p1.y, 0.0));
    pose_transform.setRotation(tf::createQuaternionFromYaw(std::atan2(p2.y - p1.y, p2.x - p1.x)));
    
    tf::Transform goal_pose_transform;
    goal_pose_transform.setOrigin(tf::Vector3(goal_pose_.x, goal_pose_.y, 0.0));
    goal_pose_transform.setRotation(tf::createQuaternionFromYaw(goal_pose_.theta));
    
    tf::Transform next_pose_shifted = goal_pose_transform * pose_transform;
    
    geometry_msgs::Pose2D next_pose;
    next_pose.x = next_pose_shifted.getOrigin().getX();
    next_pose.y = next_pose_shifted.getOrigin().getY();
    next_pose.theta = tf::getYaw(next_pose_shifted.getRotation());
    
    return next_pose;
}


geometry_msgs::Twist RobotController::getPathVelocity(ros::Time time)
{
    geometry_msgs::Pose2D pose = getPathPose(time);
    geometry_msgs::Pose2D next_pose = getPathPose(time + ros::Duration(cfg_.time_step));
    
    geometry_msgs::Twist vel;
    vel.linear.x = cfg_.path_tracking_velocity;
    vel.angular.z = distanceTheta(pose, next_pose) / cfg_.time_step;
    
    return vel;
}
    
bool RobotController::poseReached(geometry_msgs::Pose2D current_pose, 
        geometry_msgs::Pose2D target_pose)
{    
    return (   std::abs(distanceXY(current_pose, target_pose)) < cfg_.pose_tolerance_xy 
            && std::abs(distanceTheta(current_pose, target_pose)) < cfg_.pose_tolerance_theta);
}

bool RobotController::pathTrackingTimeout(ros::Time time_now)
{    
    return (time_now > path_tracking_start_time + ros::Duration(path_traversal_time));
}

void RobotController::control()
{
    ros::Time time_now = ros::Time::now();    
    geometry_msgs::Pose2D current_pose = getRobotGlobalPose(time_now);
            
    geometry_msgs::Twist cmd_vel;
    
    if(controller_state_ == ControllerState::PathTracking)
    {
        geometry_msgs::Pose2D target_pose = getPathPose(time_now);
        geometry_msgs::Twist target_vel = getPathVelocity(time_now);
        cmd_vel = pathTrackingControl(current_pose, target_pose, target_vel);
        publishNextPoseMsg(target_pose);
        
        if(pathTrackingTimeout(time_now))
        {
            controller_state_ = ControllerState::GoalPositionApproaching;
            ROS_INFO("Robot controller state: %d", int(controller_state_));
        }
    }
    else if(controller_state_ == ControllerState::GoalPositionApproaching)
    {
        cmd_vel = positionApproachingControl(current_pose, goal_pose_, time_now);
        publishNextPoseMsg(goal_pose_);
        
        if(poseReached(current_pose, goal_pose_))
        {
            controller_state_ = ControllerState::Finished;
            ROS_INFO("Robot controller state: %d", int(controller_state_));
        }
    }

    geometry_msgs::Twist safe_cmd_vel = collisionAvoidance(cmd_vel); 

    publishJointState(safe_cmd_vel);

    pub_cmd_vel_.publish(safe_cmd_vel);
    
}

// 2. Obliczenie prędkości kół
void RobotController::publishJointState(geometry_msgs::Twist cmd_vel)
{
    double left_wheel_angular_velocity = 0.0;
    double right_wheel_angular_velocity = 0.0;

    // left_wheel_angular_velocity = ?? ; // rad/s
    // right_wheel_angular_velocity = ?? ; // rad/s

    joint_state_msg_.velocity[0] = left_wheel_angular_velocity;
    joint_state_msg_.velocity[1] = right_wheel_angular_velocity;

    joint_state_msg_.position[0] += left_wheel_angular_velocity * cfg_.time_step;
    joint_state_msg_.position[1] += right_wheel_angular_velocity * cfg_.time_step;
        
    pub_joint_state_.publish(joint_state_msg_);
}

// 3. Implementacja algorytmu śledzenia trajektorii
geometry_msgs::Twist RobotController::pathTrackingControl(
    geometry_msgs::Pose2D current_pose, 
    geometry_msgs::Pose2D target_pose, 
    geometry_msgs::Twist target_vel)
{   
    geometry_msgs::Twist cmd_vel;

//    cmd_vel.linear.x = ?;
//    cmd_vel.angular.z = ?;

    return cmd_vel;
}

// 4. Implementacja algorytmu stabilizacji w punkcie
geometry_msgs::Twist RobotController::positionApproachingControl(
        geometry_msgs::Pose2D current_pose, 
        geometry_msgs::Pose2D target_pose,
        ros::Time time_now)
{
    geometry_msgs::Twist cmd_vel;
    
    geometry_msgs::Twist starget_control;
    
//    starget_control.linear.x = ?;

    cmd_vel = pathTrackingControl(current_pose, target_pose, starget_control);
    
    return cmd_vel;
}

// 5. Implementacja algorytmu unikania kolizji
geometry_msgs::Twist RobotController::collisionAvoidance(geometry_msgs::Twist cmd_vel)
{
    geometry_msgs::Twist safe_cmd_vel = cmd_vel;

    for(int i=0; i<int(laser_msg_.ranges.size()); i++)
    {
        //  Ograniczyć prędkość jeżeli spełniony jest warunek
        // if(laser_msg_.ranges[i] < ??)
        // {
        //     safe_cmd_vel.linear.x = ?;
        //     safe_cmd_vel.angular.z = ?;
        // }
    }

    return safe_cmd_vel;
}
