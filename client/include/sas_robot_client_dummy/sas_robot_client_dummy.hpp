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

#pragma once
#include <atomic>
#include <thread>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>

using namespace Eigen;
namespace sas
{

struct RobotClientDummyConfiguration
{
    std::string topic_prefix;
    double thread_sampling_time_sec;
};


class RobotClientDummy
{
private:

    RobotClientDummyConfiguration configuration_;

    //rclcpp::Subscription<sas_msgs::msg::Heartbeat>::SharedPtr subscriber_hearbeat_;
    //void _callback_heartbeat(const sas_msgs::msg::Heartbeat& msg);


    std::shared_ptr<sas::RobotDriverClient> rdi_;
    std::shared_ptr<sas::RobotDriverClient> rdi_watchdog_;
    std::shared_ptr<sas::RobotDriverClient> rdi_2_;
    std::shared_ptr<sas::RobotDriverClient> rdi_monitoring_;
    std::atomic_bool* st_break_loops_;
    std::shared_ptr<rclcpp::Node> node_;
    sas::Clock clock_;

    class Impl;
    std::unique_ptr<Impl> impl_;

    bool _should_shutdown() const;

public:

    RobotClientDummy(const RobotClientDummy&)=delete;
    RobotClientDummy()=delete;
    ~RobotClientDummy();

    RobotClientDummy(std::shared_ptr<Node> &node,
                                   const RobotClientDummyConfiguration &configuration,
                                   std::atomic_bool* break_loops);

    void control_loop();
};



}
