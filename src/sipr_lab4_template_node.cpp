#include <cstdlib>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <swrm_ros/UniqueNodeName.h>

#include "sipr_lab4_template/RobotController.h"

/*
 * 
 */
int main(int argc, char** argv) 
{
    std::string node_name = swrm_ros::uniqueNodeName("sipr_lab4_template_node");
    
    ros::init(argc, argv, node_name.c_str());
        
    std::string robot_name = "/robot_10";
    
    RobotController::Config config;
    config.robot_namespace = "/maor_driver" + robot_name;
    config.robot_frame_id = robot_name;
    config.globale_frame_id = "/ar_marker_0";
    
    config.time_step = 0.05; //seconds
    
    config.path_tracking_velocity = 0.15;
    config.path_scale = 0.3;
    
    config.pose_tolerance_xy = 0.02;
    config.pose_tolerance_theta = 2.0*M_PI/180.0;
    
//    config.k1 = ?;
//    config.k2 = ?;
//    config.k3 = ?;
//            
//    config.ksx = ?;
//    config.ksrot = ?;
//    config.s_period = ?;
    
    RobotController robot_controller(config);
    
    ros::Rate loop_rate(1.0/config.time_step);
    
    while(ros::ok())
    {
        ros::spinOnce();
        
        robot_controller.control();
        
        loop_rate.sleep();
    }

    return 0;
}

