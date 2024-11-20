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

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.hpp>
#include <mav_msgs/default_topics.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.h>
using namespace std::chrono_literals;
class HoveringExample : public rclcpp::Node
{
public:
  HoveringExample(): Node("hovering_example")
  {
    this->declare_parameter("my_parameter", "world");
    trajectory_pub =
      this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, rclcpp::SystemDefaultsQoS());

    RCLCPP_INFO_ONCE(get_logger(),"Started hovering example.");
    
    timer_ = this->create_wall_timer(1000ms, std::bind(&HoveringExample::timer_callback,this));
  }
  void  timer_callback()
  {
    std::chrono::milliseconds t = std::chrono::milliseconds(500);
    rclcpp::sleep_for(t);
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();       // CHANGE
    // request->a = atoi(argv[1]);
    // request->b = atoi(argv[2]);;
    // request->c = atoi(argv[3]);;   

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client =                // CHANGE
      create_client<std_srvs::srv::Empty>("/unpause_physics");   
    // auto unpaused =   client->async_send_request(request);
    unsigned int i = 0;
    
    // Trying to unpause Gazebo for 10 seconds.
    while (!client->wait_for_service(1s))
    {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
    // while (i <= 10 && !unpaused) {
    //   RCLCPP_INFO(get_logger(),"Wait for 1 second before trying to unpause Gazebo again.");
    //   std::this_thread::sleep_for(std::chrono::seconds(1));
    //   unpaused = rclcpp::service::call("/gazebo/unpause_physics", srv);
    //   ++i;
    // }

    trajectory_msgs::msg::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = this->now();
    Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
    double desired_yaw = 0.0;
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);
      // .c_str(),
    RCLCPP_INFO(get_logger(),"Publishing waypoint on namespace %s: [%f, %f, %f].",
            get_namespace(),            
            desired_position.x(),
            desired_position.y(),
            desired_position.z());
    this->trajectory_pub->publish(trajectory_msg);

  };
private:
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr trajectory_pub;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HoveringExample>());
  rclcpp::shutdown();

  return 0;
}
