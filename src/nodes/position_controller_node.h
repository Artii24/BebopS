/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Pasquale Oppido, University of Sannio in Benevento, Italy
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef BEBOP_CONTROL_POSITION_CONTROLLER_NODE_H
#define BEBOP_CONTROL_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_msgs/msg/actuators.hpp>
#include <mav_msgs/msg/attitude_thrust.hpp>
#include <mav_msgs/eigen_mav_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <rclcpp/callback_queue.h>
#include "rclcpp/rclcpp.hpp"
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include "bebop_simulator_r2/common.h"
#include "bebop_simulator_r2/position_controller.h"
#include "bebop_simulator_r2/parameters_rclcpp.h"
#include "bebop_simulator_r2/parameters.h"

namespace bebop_simulator {

    class PositionControllerNode{
        public:
            PositionControllerNode();
            ~PositionControllerNode();
             
            void InitializeParams();
            void Publish();

        private:
            rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("odometry");
            rclcpp::Node::SharedPtr pnh_ = rclcpp::Node::make_shared("~");

            bool waypointHasBeenPublished_ = false;

            PositionController position_controller_;

            std::string namespace_;

            //subscribers

            rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectory> cmd_multi_dof_joint_trajectory_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry::ConstPtr> odometry_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry::ConstPtr> odometry_sub_gt_;

            //publisher
            rclcpp::Publisher<mav_msgs::msg::Actuators> motor_velocity_reference_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> odometry_filtered_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> filtered_errors_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> reference_angles_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> smoothed_reference_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> uTerr_components_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> zVelocity_components_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> positionAndVelocityErrors_pub_;
            rclcpp::Publisher<nav_msgs::msg::Odometry> angularAndAngularVelocityErrors_pub_;

            nav_msgs::msg::Odometry odometry_gt_;

            void MultiDofJointTrajectoryCallback(const trajectory_msgs::msg::MultiDOFJointTrajectory::ConstPtr& trajectory_reference_msg);
            void OdometryGTCallback(const nav_msgs::msg::Odometry::ConstPtr& odometry_msg_gt);
            void OdometryCallback(const nav_msgs::msg::Odometry::ConstPtr& odometry_msg);


    };
}

#endif // BEBOP_CONTROL_POSITION_CONTROLLER_NODE_H
