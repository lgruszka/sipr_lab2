/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   RobotController.h
 * Author: maciek
 *
 * Created on 4 stycznia 2019, 15:20
 */

#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/path.hpp>

/// Pozycja (konfiguracja) robota na płaszczyźnie
struct Pose2D
{
    double x = 0.0;     /// położenie X [m]
    double y = 0.0;     /// położenie Y [m]
    double theta = 0.0; /// orientacja [rad]
};

class RobotController : public rclcpp::Node
{
public:
    
    struct Config
    {
        std::string robot_namespace;
        std::string robot_frame_id;
        std::string globale_frame_id;

        double r; /// promień koła
        double b; /// odstęp między kołami

        double time_step; /// seconds
        
        double k1, k2, k3;
        double s_period, ksx, ksrot;
        
        double path_tracking_velocity;
        double path_scale;
        
        double pose_tolerance_xy;
        double pose_tolerance_theta;
    };
    
    enum class ControllerState
    {
        Ready,
        PathTracking, 
        GoalPositionApproaching,
        Finished
    };
        
    RobotController(Config const& cfg);    
    virtual ~RobotController();
    
    void control();

private:    
    Config cfg_;
    
    ControllerState controller_state_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pose_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser_scan_;
        
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_next_pose_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_state_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;

    sensor_msgs::msg::LaserScan laser_msg_;
    Pose2D goal_pose_;
    sensor_msgs::msg::JointState joint_state_msg_;
        
    double path_traversal_time;
    rclcpp::Time path_tracking_start_time;
            
    void computePathTraversalTime();
    void publishPathMsg();
    void publishNextPoseMsg(Pose2D pose_2d);
    void publishJointState(geometry_msgs::msg::Twist cmd_vel);
    
    void laserScanCallback(sensor_msgs::msg::LaserScan const& laser_msg);
    void goalPoseCallback(geometry_msgs::msg::PoseStamped const& goal_pose_msg);
    
    Pose2D getRobotGlobalPose(rclcpp::Time time);
    
    double distanceXY(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);
    double distanceXY(Pose2D p1, Pose2D p2);
    double distanceTheta(Pose2D p1, Pose2D p2);
    geometry_msgs::msg::Point getPathPoint(double t);
    geometry_msgs::msg::Point getPathPoint(rclcpp::Time time);
    Pose2D getPathPose(rclcpp::Time time);
    geometry_msgs::msg::Twist getPathVelocity(rclcpp::Time time);
    
    bool poseReached(Pose2D current_pose,
        Pose2D target_pose);
    bool pathTrackingTimeout(rclcpp::Time time);
    
    geometry_msgs::msg::Twist pathTrackingControl(
        Pose2D current_pose,
        Pose2D target_pose,
        geometry_msgs::msg::Twist target_vel);
    
    geometry_msgs::msg::Twist positionApproachingControl(
        Pose2D current_pose,
        Pose2D goal_pose,
        rclcpp::Time time);
        
    geometry_msgs::msg::Twist collisionAvoidance(geometry_msgs::msg::Twist cmd_vel);
       

};

#endif /* ROBOTCONTROLLER_H */

