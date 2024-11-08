#ifndef INCLUDE_BEBOP_CONTROL_PARAMETERS_ROS_H_
#define INCLUDE_BEBOP_CONTROL_PARAMETERS_ROS_H_

#include "rclcpp/rclcpp.hpp"
#include <rcpputils/asserts.hpp>
#include "parameters.h"

namespace bebop_simulator {

template<typename T> inline void GetRosParameter(const rclcpp::Node::SharedPtr& nh,
                                                 const std::string& key,
                                                 const T& default_value,
                                                 T* value) {
  rcpputils::assert_true(value != nullptr);
  bool have_parameter = nh->get_parameter(key, *value);
  if (!have_parameter) {
    RCLCPP_WARN_STREAM(nh->get_logger(),"[rosparam]: could not find parameter " << nh->get_namespace()
                    << "/" << key << ", setting to default: " << default_value);
    *value = default_value;
  }
}

// The function allows to take data from the yaml file describing the quadrotor paramters
inline void GetVehicleParameters(const rclcpp::Node::SharedPtr& nh, VehicleParameters* vehicle_parameters) {
  GetRosParameter(nh, "mass",
                  vehicle_parameters->mass_,
                  &vehicle_parameters->mass_);
  GetRosParameter(nh, "inertia/xx",
                  vehicle_parameters->inertia_(0, 0),
                  &vehicle_parameters->inertia_(0, 0));
  GetRosParameter(nh, "inertia/xy",
                  vehicle_parameters->inertia_(0, 1),
                  &vehicle_parameters->inertia_(0, 1));
  vehicle_parameters->inertia_(1, 0) = vehicle_parameters->inertia_(0, 1);
  GetRosParameter(nh, "inertia/xz",
                  vehicle_parameters->inertia_(0, 2),
                  &vehicle_parameters->inertia_(0, 2));
  vehicle_parameters->inertia_(2, 0) = vehicle_parameters->inertia_(0, 2);
  GetRosParameter(nh, "inertia/yy",
                  vehicle_parameters->inertia_(1, 1),
                  &vehicle_parameters->inertia_(1, 1));
  GetRosParameter(nh, "inertia/yz",
                  vehicle_parameters->inertia_(1, 2),
                  &vehicle_parameters->inertia_(1, 2));
  vehicle_parameters->inertia_(2, 1) = vehicle_parameters->inertia_(1, 2);
  GetRosParameter(nh, "inertia/zz",
                  vehicle_parameters->inertia_(2, 2),
                  &vehicle_parameters->inertia_(2, 2));
  GetRosParameter(nh, "bf",
                  vehicle_parameters->bf_,
                  &vehicle_parameters->bf_);
  GetRosParameter(nh, "bm",
                  vehicle_parameters->bm_,
                  &vehicle_parameters->bm_);
  GetRosParameter(nh, "l",
                  vehicle_parameters->armLength_,
                  &vehicle_parameters->armLength_);
}

}

#endif /* INCLUDE_BEBOP_CONTROL_PARAMETERS_ROS_H_ */
