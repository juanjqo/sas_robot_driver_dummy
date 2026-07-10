/*
# Copyright (c) 2025 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License.
#    If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#   Based on sas_robot_driver_ur.hpp
#   (https://github.com/MarinhoLab/sas_robot_driver_ur/blob/main/include/sas_robot_driver_ur/sas_robot_driver_ur.hpp)
#
# ################################################################*/


#include "sas_robot_driver_dummy/sas_robot_driver_dummy.hpp"
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>


namespace sas
{

class RobotDriverDummy::Impl
{

public:
    DQ robot_pose_;

    VectorXd imu_orientation_;
    VectorXd imu_angular_velocity_;
    VectorXd imu_linear_acceleration_;
    Impl() : imu_orientation_(VectorXd::Zero(4)),
        imu_angular_velocity_(VectorXd::Zero(3)),
        imu_linear_acceleration_(VectorXd::Zero(3))
    {
        // Initialize orientation as identity quaternion
        imu_orientation_(0) = 1.0;  // w
        imu_orientation_(1) = 0.0;  // x
        imu_orientation_(2) = 0.0;  // y
        imu_orientation_(3) = 0.0;  // z
    }



};

RobotDriverDummy::~RobotDriverDummy()
{

}

/**
 * @brief DifferentialWheeledRobotDriver::DifferentialWheeledRobotDriver ctor of the class
 * @param configuration
 * @param break_loops
 */
void RobotDriverDummy::publish_imu(const VectorXd& orientation, const VectorXd& velocity, const VectorXd& acceleration)
{
    // Validate input sizes
    if (orientation.size() != 4) {
        throw std::runtime_error("Orientation vector must have size 4 (quaternion w,x,y,z)");
    }
    if (velocity.size() != 3) {
        throw std::runtime_error("Velocity vector must have size 3 (x,y,z)");
    }
    if (acceleration.size() != 3) {
        throw std::runtime_error("Acceleration vector must have size 3 (x,y,z)");
    }

    sensor_msgs::msg::Imu ros_msg_imu;
    ros_msg_imu.header.stamp = node_->get_clock()->now();
    ros_msg_imu.header.frame_id = "imu_link";  // or configuration_.imu_frame_id if you have it

    // Set orientation (quaternion)
    ros_msg_imu.orientation.w = orientation(0);
    ros_msg_imu.orientation.x = orientation(1);
    ros_msg_imu.orientation.y = orientation(2);
    ros_msg_imu.orientation.z = orientation(3);

    // Set orientation covariance (identity matrix for now)
    // You can set this to proper covariance values based on your sensor model
    ros_msg_imu.orientation_covariance[0] = 0.001;
    ros_msg_imu.orientation_covariance[4] = 0.001;
    ros_msg_imu.orientation_covariance[8] = 0.001;

    // Set angular velocity
    ros_msg_imu.angular_velocity.x = velocity(0);
    ros_msg_imu.angular_velocity.y = velocity(1);
    ros_msg_imu.angular_velocity.z = velocity(2);

    // Set angular velocity covariance
    ros_msg_imu.angular_velocity_covariance[0] = 0.01;
    ros_msg_imu.angular_velocity_covariance[4] = 0.01;
    ros_msg_imu.angular_velocity_covariance[8] = 0.01;

    // Set linear acceleration
    ros_msg_imu.linear_acceleration.x = acceleration(0);
    ros_msg_imu.linear_acceleration.y = acceleration(1);
    ros_msg_imu.linear_acceleration.z = acceleration(2);

    // Set linear acceleration covariance
    ros_msg_imu.linear_acceleration_covariance[0] = 0.01;
    ros_msg_imu.linear_acceleration_covariance[4] = 0.01;
    ros_msg_imu.linear_acceleration_covariance[8] = 0.01;

    publisher_IMU_state_->publish(ros_msg_imu);
}

RobotDriverDummy::RobotDriverDummy(std::shared_ptr<Node> &node,
                                                               const Configuration &configuration,
                                                               std::atomic_bool *break_loops):
    RobotDriver(break_loops), configuration_(configuration),
    node_{node}
{
    impl_ = std::make_unique<RobotDriverDummy::Impl>();
   

    VectorXd joint_limits_min = Eigen::VectorXd::Constant(3, -std::numeric_limits<double>::infinity());
    VectorXd joint_limits_max = Eigen::VectorXd::Constant(3,  std::numeric_limits<double>::infinity());
    joint_limits_ = {joint_limits_min, joint_limits_max};

    //rdi_ = std::make_shared<sas::RobotDriverClient>(node_, "/watchdog_commander/");

    // Set the callback using the public method

    publisher_IMU_state_ = node_->create_publisher<sensor_msgs::msg::Imu>(configuration.topic_prefix + "/get/IMU_state", 1);


    set_control_loop_callback([this]() {

        // Static local variables - persist between iterations
        static int iteration_counter = 0;
        static double accumulated_time = 0.0;

        iteration_counter++;
        accumulated_time += configuration_.thread_sampling_time_sec;

        if (iteration_counter % 100 == 0) {
            RCLCPP_INFO_STREAM(node_->get_logger(),
                               "Iteration: " << iteration_counter
                                             << ", Accumulated time: " << accumulated_time << "s");
        }

        double time = accumulated_time;
        impl_->imu_orientation_(0) = std::cos(time * 0.1);  // w
        impl_->imu_orientation_(1) = 0.0;                   // x
        impl_->imu_orientation_(2) = 0.0;                   // y
        impl_->imu_orientation_(3) = std::sin(time * 0.1);  // z

        // Simulate angular velocity
        impl_->imu_angular_velocity_(0) = 0.1 * std::cos(time * 0.5);
        impl_->imu_angular_velocity_(1) = 0.05 * std::sin(time * 0.7);
        impl_->imu_angular_velocity_(2) = 0.2 * std::cos(time * 0.3);

        // Simulate linear acceleration (gravity + motion)
        impl_->imu_linear_acceleration_(0) = 0.1 * std::sin(time * 0.2);
        impl_->imu_linear_acceleration_(1) = 0.1 * std::cos(time * 0.4);
        impl_->imu_linear_acceleration_(2) = 9.81;  // gravity
        publish_imu(impl_->imu_orientation_, impl_->imu_angular_velocity_, impl_->imu_linear_acceleration_);


    });


}


/**
 * @brief DifferentialWheeledRobotRobotDriver::get_joint_positions returns the robot configuration.
 * @return A vector 3x1 containing the robot configuration. E.g., q=[x,y, phi].
 */
VectorXd RobotDriverDummy::get_joint_positions()
{
    return (VectorXd(3)<< 0, 0, 0).finished();
}

void RobotDriverDummy::set_target_joint_positions([[maybe_unused]] const VectorXd& desired_joint_positions_rad)
{

}

/**
 * @brief DifferentialWheeledRobotRobotDriver::set_target_joint_velocities sets the target task-space velocities of the mobile platform. The
 *                      target velocities are expressed in the body frame.
 * @param target_velocities A vector of 3x1 containing the desired velocities. E.g, u = [x_dot, y_dot, phi_dot].
 */
void RobotDriverDummy::set_target_joint_velocities([[maybe_unused]] const VectorXd &target_velocities)
{



}


VectorXd RobotDriverDummy::get_joint_velocities()
{
    //throw std::runtime_error(std::string(__FUNCTION__)+" is not available.");
    return VectorXd::Zero(3);
}


VectorXd RobotDriverDummy::get_joint_torques()
{
    return VectorXd::Zero(3);
}


/**
 * @brief DifferentialWheeledRobotRobotDriver::connect
 */
void RobotDriverDummy::connect()
{
    
}


/**
 * @brief DifferentialWheeledRobotRobotDriver::disconnect
 */
void RobotDriverDummy::disconnect()
{
    for (int i=0;i<5;i++)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "disconnecting...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


/**
 * @brief DifferentialWheeledRobotRobotDriver::initialize
 */
void RobotDriverDummy::initialize()
{
    /*
    if (configuration_.watchdog_period != 0.0 and configuration_.watchdog_period > 0.0)
    {
        const std::chrono::nanoseconds period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(configuration_.watchdog_period));
        //watchdog_start(period);
        //watchdog_trigger(std::chrono::system_clock::now());
    }
*/
}

/**
 * @brief DifferentialWheeledRobotRobotDriver::deinitialize
 */
void RobotDriverDummy::deinitialize()
{
    for (int i=0;i<5;i++)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(), "deinitializing...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}




}
