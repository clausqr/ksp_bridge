#pragma once

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <krpc/services/space_center.hpp>
#include <string>

template <typename T>
T clamp(T val, T min, T max)
{
    if (val < min) {
        return min;
    } else if (val > max) {
        return max;
    } else {
        return val;
    }
}

const char* base_name(const char*);

std::string str_lowercase(const std::string&);

geometry_msgs::msg::Vector3 tuple2vector3(const std::tuple<double, double, double>&);

geometry_msgs::msg::Quaternion tuple2quaternion(const std::tuple<double, double, double, double>&);

geometry_msgs::msg::TransformStamped get_transform(krpc::services::SpaceCenter&, std::tuple<double, double, double>, std::tuple<double, double, double, double>, krpc::services::SpaceCenter::ReferenceFrame, krpc::services::SpaceCenter::ReferenceFrame);

// Boundary between KSP's native vessel frame (left-handed: x=right,
// y=forward, z=down) and aerospace FRD body frame (right-handed:
// x=forward, y=right, z=down). Callers pass a quantity expressed in the
// KSP vessel frame and receive the same physical quantity expressed in
// FRD. Only valid for vessel-body-frame inputs; do not apply to
// celestial-body-frame vectors such as kerbin-frame velocity or position.
geometry_msgs::msg::Vector3 vessel_frd_vector(const geometry_msgs::msg::Vector3&);
geometry_msgs::msg::Vector3 vessel_frd_pseudo_vector(const geometry_msgs::msg::Vector3&);
geometry_msgs::msg::Quaternion vessel_frd_quaternion(const geometry_msgs::msg::Quaternion&);
