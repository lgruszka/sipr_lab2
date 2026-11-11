#include <cstdlib>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "sipr_lab2/RobotController.h"

/*
 *
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    RobotController::Config config;

    config.robot_frame_id = "base_link";
    config.globale_frame_id = "map";

    // Parametry odtwarzania trajektorii
    config.time_step = 0.1; // sekundy

    config.path_tracking_velocity = 1.0;
    config.path_scale = 3;

    config.pose_tolerance_xy = 0.2;
    config.pose_tolerance_theta = 2.0*M_PI/180.0;

    // Węzeł z obiektem kontrolera ruchu implementującym
    // algorytmy sterowania; co config.time_step sekund
    // wywołuje pojedynczy krok algorytmu sterowania
    auto robot_controller = std::make_shared<RobotController>(config);

    rclcpp::spin(robot_controller);

    rclcpp::shutdown();

    return 0;
}
