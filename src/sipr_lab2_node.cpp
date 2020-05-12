#include <cstdlib>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose2D.h>

#include "sipr_lab2/RobotController.h"

/*
 *
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "sipr_lab2_node", ros::init_options::AnonymousName);

    RobotController::Config config;

    config.robot_frame_id = "base_link";
    config.globale_frame_id = "map";

    // Parametry odtwarzania trajektorii
    config.time_step = 0.1; // sekundy

    config.path_tracking_velocity = 1.0;
    config.path_scale = 3;

    config.pose_tolerance_xy = 0.2;
    config.pose_tolerance_theta = 2.0*M_PI/180.0;

    // Obiekt kontrolera ruchu implementujący
    // algorytmy sterowania 
    RobotController robot_controller(config);

    // Częstotliwość działania węzła
    ros::Rate loop_rate(1.0/config.time_step);

    while(ros::ok())
    {
        ros::spinOnce();

        // Pojedynczy krok algorytmu sterowania
        robot_controller.control();

        loop_rate.sleep();
    }

    return 0;
}
