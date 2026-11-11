#include "sipr_lab2/RobotController.h"

#include <cmath>

#include <angles/angles.h>
#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


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

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
}

RobotController::RobotController(Config const& cfg) : rclcpp::Node("sipr_lab2_node"), cfg_(cfg)
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
    path_tracking_start_time = now();
    RCLCPP_INFO(get_logger(), "Robot controller state: %d", int(controller_state_));
    
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    sub_goal_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            cfg_.robot_namespace+"/goal", 1,
            [this](geometry_msgs::msg::PoseStamped const& msg) {goalPoseCallback(msg);});

    sub_laser_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
            cfg_.robot_namespace+"/base_scan", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::LaserScan const& msg) {laserScanCallback(msg);});

    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>(cfg_.robot_namespace+"/cmd_vel", 1);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(cfg_.robot_namespace+"/path", 1);
    pub_next_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(cfg_.robot_namespace+"/next_pose", 1);
    pub_joint_state_ = create_publisher<sensor_msgs::msg::JointState>(cfg_.robot_namespace+"/wheel_joint_states", 1);

    // Wywoływanie metody control() co cfg_.time_step sekund
    control_timer_ = create_wall_timer(
            std::chrono::duration<double>(cfg_.time_step),
            [this]() {control();});
}

RobotController::~RobotController() 
{
    
}

Pose2D RobotController::getRobotGlobalPose(rclcpp::Time time_now)
{
    Pose2D pose;
    
    //Pobranie transformacji między układem globalnym a układem robota
    try 
    {
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(cfg_.time_step/2.0);
        
        geometry_msgs::msg::TransformStamped transform_box_robot = tf_buffer_->lookupTransform(
                cfg_.globale_frame_id, cfg_.robot_frame_id,
                time_now, timeout);
        
        pose.x = transform_box_robot.transform.translation.x;
        pose.y = transform_box_robot.transform.translation.y;
        pose.theta = tf2::getYaw(transform_box_robot.transform.rotation);
        
    } catch (tf2::TransformException& ex)
    {
        RCLCPP_WARN(get_logger(), "TF exception:\n%s", ex.what());
    }
    
    return pose;
}

void RobotController::laserScanCallback(sensor_msgs::msg::LaserScan const& laser_msg)
{ 
    laser_msg_ = laser_msg;
}

void RobotController::goalPoseCallback(geometry_msgs::msg::PoseStamped const& goal_pose_msg)
{
    goal_pose_.x = goal_pose_msg.pose.position.x;
    goal_pose_.y = goal_pose_msg.pose.position.y;
    goal_pose_.theta = tf2::getYaw(goal_pose_msg.pose.orientation);
    
    path_tracking_start_time = now();
    publishPathMsg();
    
    controller_state_ = ControllerState::PathTracking;
    RCLCPP_INFO(get_logger(), "Robot controller state: %d", int(controller_state_));
}

void RobotController::computePathTraversalTime()
{
    double distance = 0.0;
    int steps = 1000;
    
    geometry_msgs::msg::Point p1 = getPathPoint(0.0);
    
    for(int i=1; i<=steps; i++)
    {
        geometry_msgs::msg::Point p2 = getPathPoint( double(i)/steps );
        
        distance += distanceXY(p1, p2);
        
        p1 = p2;
    }
    
    path_traversal_time = distance / cfg_.path_tracking_velocity;
    
    RCLCPP_INFO(get_logger(), "Estimated path traversal time: %f", path_traversal_time);
}

void RobotController::publishPathMsg()
{
    nav_msgs::msg::Path path;
    path.header.frame_id = cfg_.globale_frame_id;
    path.header.stamp = path_tracking_start_time;
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = cfg_.globale_frame_id;
    
    for(double time = 0.0; time < path_traversal_time; time+=cfg_.time_step)
    {        
        rclcpp::Time pose_time = path_tracking_start_time + rclcpp::Duration::from_seconds(time);
        pose.header.stamp = pose_time;
        Pose2D pose_2d = getPathPose(pose_time);
        pose.pose.position.x = pose_2d.x;
        pose.pose.position.y = pose_2d.y;
        pose.pose.orientation = createQuaternionMsgFromYaw(pose_2d.theta);
        
        path.poses.push_back(pose);
    }
    
    pub_path_->publish(path);
}


void RobotController::publishNextPoseMsg(Pose2D pose_2d)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = cfg_.globale_frame_id;
    pose.header.stamp = now();
    pose.pose.position.x = pose_2d.x;
    pose.pose.position.y = pose_2d.y;
    pose.pose.orientation = createQuaternionMsgFromYaw(pose_2d.theta);
    
    pub_next_pose_->publish(pose);
}


double RobotController::distanceXY(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double RobotController::distanceXY(Pose2D p1, Pose2D p2)
{
    return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double RobotController::distanceTheta(Pose2D p1, Pose2D p2)
{    
    double angular_distance = angles::shortest_angular_distance(p1.theta, p2.theta); // najkrótsza odległość między kątami < -PI, PI >
    
    return angular_distance;
}

geometry_msgs::msg::Point RobotController::getPathPoint(double t)
{
    geometry_msgs::msg::Point point;
    
    point.x = cfg_.path_scale * std::sin(t * 4.0 * M_PI);
    point.y = cfg_.path_scale * (std::sin(t / 2.0 * 4.0 * M_PI) + std::cos(2.0 * t * 4.0 * M_PI)/7.0);
    
    return point;    
}

geometry_msgs::msg::Point RobotController::getPathPoint(rclcpp::Time time)
{
    rclcpp::Duration d = (time - path_tracking_start_time);
    double t = d.seconds() / path_traversal_time;
        
    geometry_msgs::msg::Point point = getPathPoint(t);
        
    return point;    
}

Pose2D RobotController::getPathPose(rclcpp::Time time_now)
{
    geometry_msgs::msg::Point p1 = getPathPoint(time_now);
    geometry_msgs::msg::Point p2 = getPathPoint(time_now + rclcpp::Duration::from_seconds(cfg_.time_step));
    
    tf2::Transform pose_transform;
    pose_transform.setOrigin(tf2::Vector3(p1.x, p1.y, 0.0));
    tf2::Quaternion pose_rotation;
    pose_rotation.setRPY(0.0, 0.0, std::atan2(p2.y - p1.y, p2.x - p1.x));
    pose_transform.setRotation(pose_rotation);
    
    tf2::Transform goal_pose_transform;
    goal_pose_transform.setOrigin(tf2::Vector3(goal_pose_.x, goal_pose_.y, 0.0));
    tf2::Quaternion goal_rotation;
    goal_rotation.setRPY(0.0, 0.0, goal_pose_.theta);
    goal_pose_transform.setRotation(goal_rotation);
    
    tf2::Transform next_pose_shifted = goal_pose_transform * pose_transform;
    
    Pose2D next_pose;
    next_pose.x = next_pose_shifted.getOrigin().getX();
    next_pose.y = next_pose_shifted.getOrigin().getY();
    next_pose.theta = tf2::getYaw(next_pose_shifted.getRotation());
    
    return next_pose;
}


geometry_msgs::msg::Twist RobotController::getPathVelocity(rclcpp::Time time)
{
    Pose2D pose = getPathPose(time);
    Pose2D next_pose = getPathPose(time + rclcpp::Duration::from_seconds(cfg_.time_step));
    
    geometry_msgs::msg::Twist vel;
    vel.linear.x = cfg_.path_tracking_velocity;
    vel.angular.z = distanceTheta(pose, next_pose) / cfg_.time_step;
    
    return vel;
}
    
bool RobotController::poseReached(Pose2D current_pose,
        Pose2D target_pose)
{    
    return (   std::abs(distanceXY(current_pose, target_pose)) < cfg_.pose_tolerance_xy 
            && std::abs(distanceTheta(current_pose, target_pose)) < cfg_.pose_tolerance_theta);
}

bool RobotController::pathTrackingTimeout(rclcpp::Time time_now)
{    
    return (time_now > path_tracking_start_time + rclcpp::Duration::from_seconds(path_traversal_time));
}

void RobotController::control()
{
    rclcpp::Time time_now = now();
    Pose2D current_pose = getRobotGlobalPose(time_now);
            
    geometry_msgs::msg::Twist cmd_vel;
    
    if(controller_state_ == ControllerState::PathTracking)
    {
        Pose2D target_pose = getPathPose(time_now);
        geometry_msgs::msg::Twist target_vel = getPathVelocity(time_now);
        cmd_vel = pathTrackingControl(current_pose, target_pose, target_vel);
        publishNextPoseMsg(target_pose);
        
        if(pathTrackingTimeout(time_now))
        {
            controller_state_ = ControllerState::GoalPositionApproaching;
            RCLCPP_INFO(get_logger(), "Robot controller state: %d", int(controller_state_));
        }
    }
    else if(controller_state_ == ControllerState::GoalPositionApproaching)
    {
        cmd_vel = positionApproachingControl(current_pose, goal_pose_, time_now);
        publishNextPoseMsg(goal_pose_);
        
        if(poseReached(current_pose, goal_pose_))
        {
            controller_state_ = ControllerState::Finished;
            RCLCPP_INFO(get_logger(), "Robot controller state: %d", int(controller_state_));
        }
    }

    geometry_msgs::msg::Twist safe_cmd_vel = collisionAvoidance(cmd_vel);

    publishJointState(safe_cmd_vel);

    pub_cmd_vel_->publish(safe_cmd_vel);
    
}

// 2. Obliczenie prędkości kół
void RobotController::publishJointState(geometry_msgs::msg::Twist cmd_vel)
{
    double left_wheel_angular_velocity = 0.0;
    double right_wheel_angular_velocity = 0.0;

    // left_wheel_angular_velocity = ?? ; // rad/s
    // right_wheel_angular_velocity = ?? ; // rad/s

    joint_state_msg_.header.stamp = now();
    joint_state_msg_.velocity[0] = left_wheel_angular_velocity;
    joint_state_msg_.velocity[1] = right_wheel_angular_velocity;

    joint_state_msg_.position[0] += left_wheel_angular_velocity * cfg_.time_step;
    joint_state_msg_.position[1] += right_wheel_angular_velocity * cfg_.time_step;
        
    pub_joint_state_->publish(joint_state_msg_);
}

// 3. Implementacja algorytmu śledzenia trajektorii
geometry_msgs::msg::Twist RobotController::pathTrackingControl(
    Pose2D current_pose,
    Pose2D target_pose,
    geometry_msgs::msg::Twist target_vel)
{   
    geometry_msgs::msg::Twist cmd_vel;

//    cmd_vel.linear.x = ?;
//    cmd_vel.angular.z = ?;

    return cmd_vel;
}

// 4. Implementacja algorytmu stabilizacji w punkcie
geometry_msgs::msg::Twist RobotController::positionApproachingControl(
        Pose2D current_pose,
        Pose2D target_pose,
        rclcpp::Time time_now)
{
    geometry_msgs::msg::Twist cmd_vel;
    
    geometry_msgs::msg::Twist starget_control;
    
//    starget_control.linear.x = ?;

    cmd_vel = pathTrackingControl(current_pose, target_pose, starget_control);
    
    return cmd_vel;
}

// 5. Implementacja algorytmu unikania kolizji
geometry_msgs::msg::Twist RobotController::collisionAvoidance(geometry_msgs::msg::Twist cmd_vel)
{
    geometry_msgs::msg::Twist safe_cmd_vel = cmd_vel;

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
