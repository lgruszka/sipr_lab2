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

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <sensor_msgs/Range.h>
#include <nav_msgs/Path.h>

class RobotController 
{
public:
    
    struct Config
    {
        std::string robot_namespace;
        std::string robot_frame_id;
        std::string globale_frame_id;        
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
    
    ros::NodeHandle nh_;
    tf::TransformListener tf_;
    
    ros::Subscriber sub_goal_pose_;
    
    ros::Subscriber sub_left_sonar_;
    ros::Subscriber sub_right_sonar_;
        
    ros::Publisher pub_cmd_vel_;    
    ros::Publisher pub_path_;     
    ros::Publisher pub_next_pose_;  
    
    sensor_msgs::Range left_sonar_;
    sensor_msgs::Range right_sonar_;
    geometry_msgs::Pose2D goal_pose_;
        
    double path_traversal_time;
    ros::Time path_tracking_start_time;
            
    void computePathTraversalTime();
    void publishPathMsg();
    void publishNextPoseMsg(geometry_msgs::Pose2D pose_2d);
    
    void leftSonarCallback(sensor_msgs::Range const& range_msg);
    void rightSonarCallback(sensor_msgs::Range const& range_msg);
    void goalPoseCallback(geometry_msgs::PoseStamped const& goal_pose_msg);
    
    geometry_msgs::Pose2D getRobotGlobalPose(ros::Time time);
    
    double distanceXY(geometry_msgs::Point p1, geometry_msgs::Point p2);
    double distanceXY(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2);
    double distanceTheta(geometry_msgs::Pose2D p1, geometry_msgs::Pose2D p2);
    geometry_msgs::Point getPathPoint(double t);    
    geometry_msgs::Point getPathPoint(ros::Time time);    
    geometry_msgs::Pose2D getPathPose(ros::Time time);  
    geometry_msgs::Twist getPathVelocity(ros::Time time);
    
    bool poseReached(geometry_msgs::Pose2D current_pose, 
        geometry_msgs::Pose2D target_pose);    
    bool pathTrackingTimeout(ros::Time time);
    
    geometry_msgs::Twist positionApproachingControl(
        geometry_msgs::Pose2D current_pose, 
        geometry_msgs::Pose2D goal_pose,
        ros::Time time);
    
    geometry_msgs::Twist pathTrackingControl(
        geometry_msgs::Pose2D current_pose, 
        geometry_msgs::Pose2D target_pose, 
        geometry_msgs::Twist target_vel);
        
    geometry_msgs::Twist collisionAvoidance(geometry_msgs::Twist cmd_vel);
       

};

#endif /* ROBOTCONTROLLER_H */

