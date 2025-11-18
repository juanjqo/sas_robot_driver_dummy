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
    Impl()
    {

    };



};

RobotDriverDummy::~RobotDriverDummy()
{

}

/**
 * @brief DifferentialWheeledRobotDriver::DifferentialWheeledRobotDriver ctor of the class
 * @param configuration
 * @param break_loops
 */
RobotDriverDummy::RobotDriverDummy(std::shared_ptr<Node> &node,
                                                               const DifferentialWheeledRobotConfiguration &configuration,
                                                               std::atomic_bool *break_loops):
    RobotDriver(break_loops), configuration_(configuration),
    node_{node}
{
    impl_ = std::make_unique<RobotDriverDummy::Impl>();
   

    VectorXd joint_limits_min = Eigen::VectorXd::Constant(3, -std::numeric_limits<double>::infinity());
    VectorXd joint_limits_max = Eigen::VectorXd::Constant(3,  std::numeric_limits<double>::infinity());
    joint_limits_ = {joint_limits_min, joint_limits_max};

    //rdi_ = std::make_shared<sas::RobotDriverClient>(node_, "/watchdog_commander/");

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
