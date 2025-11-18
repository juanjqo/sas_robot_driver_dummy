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


#include "sas_robot_client_dummy/sas_robot_client_dummy.hpp"
#include <memory>
#include <sas_core/eigen3_std_conversions.hpp>


namespace sas
{

class RobotClientDummy::Impl
{

public:
    DQ robot_pose_;
    Impl()
    {

    };



};

RobotClientDummy::~RobotClientDummy()
{

}

/**
 * @brief DifferentialWheeledRobotDriver::DifferentialWheeledRobotDriver ctor of the class
 * @param configuration
 * @param break_loops
 */
RobotClientDummy::RobotClientDummy(std::shared_ptr<Node> &node,
                                   const RobotClientDummyConfiguration &configuration,
                                   std::atomic_bool *break_loops):
    configuration_(configuration),
    st_break_loops_{break_loops},
    node_{node},
    clock_{configuration.thread_sampling_time_sec}
{
    impl_ = std::make_unique<RobotClientDummy::Impl>();
   
    // This client is not allowed to command the watchdog. Therefore, this feature is blacklisted
    // Therefore, this rdi_->send_watchdog_trigger(false) does not have any effect
    std::vector<RobotDriverClient::MODE_BLACKLIST_FLAG> flags;
    flags.push_back(RobotDriverClient::MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL);
    rdi_ = std::make_shared<sas::RobotDriverClient>(node_, configuration_.topic_prefix, flags);



    // This client is not allowed to command the joints. Therefore, this feature is blacklisted
    std::vector<RobotDriverClient::MODE_BLACKLIST_FLAG> flags2;
    flags2.push_back(RobotDriverClient::MODE_BLACKLIST_FLAG::JOINT_CONTROL);
    rdi_watchdog_ = std::make_shared<sas::RobotDriverClient>(node_, configuration_.topic_prefix, flags2);


    // This client is not allowed to command the joints or the watchdog. Therefore, these features are blacklisted
    std::vector<RobotDriverClient::MODE_BLACKLIST_FLAG> flags3;
    flags3.push_back(RobotDriverClient::MODE_BLACKLIST_FLAG::JOINT_CONTROL);
    flags3.push_back(RobotDriverClient::MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL);
    rdi_monitoring_ = std::make_shared<sas::RobotDriverClient>(node_, configuration_.topic_prefix, flags3);


    // This is done to test if an exception is thrown.
    // rdi_2_ = std::make_shared<sas::RobotDriverClient>(node_, configuration_.topic_prefix, flags);


}

void RobotClientDummy::control_loop()
{
    try {
        clock_.init();
        rclcpp::spin_some(node_);

        while (!rdi_->is_enabled() && !_should_shutdown())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for control client! ");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
        }
        while (!rdi_watchdog_->is_enabled() && !_should_shutdown())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for watchdog client! ");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
        }
        while (!rdi_monitoring_->is_enabled() && !_should_shutdown())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Waiting for monitoring client! ");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
        }




        RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Clients ready! ");
        while(!_should_shutdown())
        {
            RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "::Running control loop! ");
            rclcpp::spin_some(node_);
            clock_.update_and_sleep();
            rclcpp::spin_some(node_);
            rdi_->send_target_joint_positions(rdi_->get_joint_positions());

            rdi_watchdog_->send_watchdog_trigger(true);
            rdi_->send_watchdog_trigger(false);
            rdi_monitoring_->get_joint_positions();

        }
    }
    catch(const std::exception& e)
    {
        std::cerr<<"::Exception caught::" << e.what() <<std::endl;
    }
}


bool RobotClientDummy::_should_shutdown() const
{
    return (*st_break_loops_);
}




}
