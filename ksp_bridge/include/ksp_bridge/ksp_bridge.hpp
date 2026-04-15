#pragma once

#include <krpc.hpp>
#include <krpc/services/krpc.hpp>
#include <krpc/services/space_center.hpp>
#include <krpc/stream.hpp>
#include <ksp_bridge_interfaces/msg/celestial_bodies.hpp>
#include <ksp_bridge_interfaces/msg/cmd_rotation.hpp>
#include <ksp_bridge_interfaces/msg/cmd_throttle.hpp>
#include <ksp_bridge_interfaces/msg/control.hpp>
#include <ksp_bridge_interfaces/msg/flight.hpp>
#include <ksp_bridge_interfaces/msg/orbit.hpp>
#include <ksp_bridge_interfaces/msg/parts.hpp>
#include <ksp_bridge_interfaces/msg/vessel.hpp>
#include <ksp_bridge_interfaces/srv/activation.hpp>
#include <ksp_bridge_interfaces/srv/sas.hpp>
#include <ksp_bridge_interfaces/srv/string.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class KSPBridge : public rclcpp::Node {
public:
    KSPBridge();

private:
    void connect();
    bool is_valid_screen();

    void validate_active_vessel();
    void invalidate_active_vessel();
    void find_active_vessel();
    void init_interfaces();
    void init_celestial_bodies();

    bool change_reference_frame(const std::string& name);

    struct NamedReferenceFrame {
        std::mutex lock;
        std::string name;
        krpc::services::SpaceCenter::ReferenceFrame refrence_frame;
    };

    // Fast-group telemetry streams. Bundled behind a unique_ptr so the whole
    // set can be dropped on vessel invalidation / reference-frame change and
    // rebuilt lazily on the next publish_fast tick.
    struct FastStreams {
        // vessel (frame-independent)
        krpc::Stream<std::string> vessel_name;
        krpc::Stream<krpc::services::SpaceCenter::VesselType> vessel_type;
        krpc::Stream<krpc::services::SpaceCenter::VesselSituation> vessel_situation;
        krpc::Stream<bool> vessel_recoverable;
        krpc::Stream<double> vessel_met;
        krpc::Stream<std::string> vessel_biome;
        krpc::Stream<int32_t> vessel_crew_capacity;
        krpc::Stream<int32_t> vessel_crew_count;
        krpc::Stream<float> vessel_mass;
        krpc::Stream<float> vessel_dry_mass;
        krpc::Stream<float> vessel_thrust;
        krpc::Stream<float> vessel_available_thrust;
        krpc::Stream<float> vessel_max_thrust;
        krpc::Stream<float> vessel_max_vacuum_thrust;
        krpc::Stream<float> vessel_specific_impulse;
        krpc::Stream<float> vessel_vacuum_specific_impulse;
        krpc::Stream<float> vessel_kerbin_sea_level_specific_impulse;
        krpc::Stream<std::tuple<double, double, double>> vessel_moment_of_inertia;
        krpc::Stream<std::vector<double>> vessel_inertia_tensor;
        // vessel (frame-dependent)
        krpc::Stream<std::tuple<double, double, double>> vessel_position;
        krpc::Stream<std::tuple<double, double, double>> vessel_velocity;
        krpc::Stream<std::tuple<double, double, double, double>> vessel_rotation;
        krpc::Stream<std::tuple<double, double, double>> vessel_direction;
        krpc::Stream<std::tuple<double, double, double>> vessel_angular_velocity;
        // flight (entire Flight object is frame-bound via vessel.flight(rf))
        krpc::Stream<float> flight_g_force;
        krpc::Stream<double> flight_mean_altitude;
        krpc::Stream<double> flight_surface_altitude;
        krpc::Stream<double> flight_bedrock_altitude;
        krpc::Stream<std::tuple<double, double, double>> flight_velocity;
        krpc::Stream<double> flight_speed;
        krpc::Stream<double> flight_horizontal_speed;
        krpc::Stream<double> flight_vertical_speed;
        krpc::Stream<std::tuple<double, double, double>> flight_center_of_mass;
        krpc::Stream<std::tuple<double, double, double, double>> flight_rotation;
        krpc::Stream<std::tuple<double, double, double>> flight_direction;
        krpc::Stream<float> flight_pitch;
        krpc::Stream<float> flight_heading;
        krpc::Stream<float> flight_roll;
        krpc::Stream<std::tuple<double, double, double>> flight_prograde;
        krpc::Stream<std::tuple<double, double, double>> flight_retrograde;
        krpc::Stream<std::tuple<double, double, double>> flight_normal;
        krpc::Stream<std::tuple<double, double, double>> flight_anti_normal;
        krpc::Stream<std::tuple<double, double, double>> flight_radial;
        krpc::Stream<std::tuple<double, double, double>> flight_anti_radial;
        krpc::Stream<float> flight_atmosphere_density;
        krpc::Stream<float> flight_dynamic_pressure;
        krpc::Stream<float> flight_static_pressure;
        krpc::Stream<float> flight_static_pressure_at_msl;
        krpc::Stream<std::tuple<double, double, double>> flight_aerodynamic_force;
        krpc::Stream<std::tuple<double, double, double>> flight_lift;
        krpc::Stream<std::tuple<double, double, double>> flight_drag;
        krpc::Stream<float> flight_speed_of_sound;
        krpc::Stream<float> flight_mach;
        krpc::Stream<float> flight_true_air_speed;
        krpc::Stream<float> flight_equivalent_air_speed;
        krpc::Stream<float> flight_terminal_velocity;
        krpc::Stream<float> flight_angle_of_attack;
        krpc::Stream<float> flight_sideslip_angle;
        krpc::Stream<float> flight_total_air_temperature;
        krpc::Stream<float> flight_static_air_temperature;
        // orbit (frame-independent)
        krpc::Stream<double> orbit_apoapsis;
        krpc::Stream<double> orbit_periapsis;
        krpc::Stream<double> orbit_apoapsis_altitude;
        krpc::Stream<double> orbit_periapsis_altitude;
        krpc::Stream<double> orbit_semi_major_axis;
        krpc::Stream<double> orbit_semi_minor_axis;
        krpc::Stream<double> orbit_radius;
        krpc::Stream<double> orbit_speed;
        krpc::Stream<double> orbit_period;
        krpc::Stream<double> orbit_time_to_apoapsis;
        krpc::Stream<double> orbit_time_to_periapsis;
        krpc::Stream<double> orbit_eccentricity;
        krpc::Stream<double> orbit_inclination;
        krpc::Stream<double> orbit_longitude_of_ascending_node;
        krpc::Stream<double> orbit_argument_of_periapsis;
        krpc::Stream<double> orbit_mean_anomaly_at_epoch;
        krpc::Stream<double> orbit_epoch;
        krpc::Stream<double> orbit_mean_anomaly;
        krpc::Stream<double> orbit_eccentric_anomaly;
        krpc::Stream<double> orbit_true_anomaly;
        krpc::Stream<double> orbit_orbital_speed;
        krpc::Stream<double> orbit_time_to_soi_change;
        // orbit SOI body name cached at setup; refreshed on invalidate/frame change
        std::string orbit_body_name;
    };

    std::unique_ptr<FastStreams> m_fast_streams;
    void setup_fast_streams(NamedReferenceFrame& frame);
    void teardown_fast_streams();

    std::unique_ptr<krpc::Client> m_ksp_client;
    std::unique_ptr<krpc::services::KRPC> m_krpc;
    std::unique_ptr<krpc::services::SpaceCenter> m_space_center;
    std::vector<std::string> m_param_celestial_bodies;
    std::unique_ptr<krpc::services::SpaceCenter::Vessel> m_vessel;
    NamedReferenceFrame m_refrence_frame;
    std::map<std::string, krpc::services::SpaceCenter::CelestialBody> m_celestial_bodies;

    // publishers
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Vessel>::SharedPtr m_vessel_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Control>::SharedPtr m_control_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Flight>::SharedPtr m_flight_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Parts>::SharedPtr m_parts_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::CelestialBodies>::SharedPtr m_celestial_bodies_publisher;
    rclcpp::Publisher<ksp_bridge_interfaces::msg::Orbit>::SharedPtr m_orbit_publisher;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;

    rclcpp::TimerBase::SharedPtr m_fast_timer;
    rclcpp::TimerBase::SharedPtr m_parts_timer;
    rclcpp::TimerBase::SharedPtr m_bodies_timer;

    ksp_bridge_interfaces::msg::Vessel m_vessel_data;
    ksp_bridge_interfaces::msg::Control m_control_data;
    ksp_bridge_interfaces::msg::Flight m_flight_data;
    ksp_bridge_interfaces::msg::Parts m_parts_data;
    ksp_bridge_interfaces::msg::CelestialBodies m_celestial_bodies_data;
    ksp_bridge_interfaces::msg::Orbit m_orbit_data;

    bool gather_vessel_data(NamedReferenceFrame& frame);
    bool gather_control_data(NamedReferenceFrame& frame);
    bool gather_flight_data(NamedReferenceFrame& frame);
    bool gather_parts_data();
    bool gather_celestial_bodies_data(NamedReferenceFrame& frame);
    bool gather_orbit_data();

    void send_tf_tree(NamedReferenceFrame& frame);

    void publish_fast();
    void publish_parts();
    void publish_bodies();

    // subscribers
    rclcpp::Subscription<ksp_bridge_interfaces::msg::CmdThrottle>::SharedPtr m_cmd_throttle_sub;
    rclcpp::Subscription<ksp_bridge_interfaces::msg::CmdRotation>::SharedPtr m_cmd_rotation_sub;

    void cmd_throttle_sub(const ksp_bridge_interfaces::msg::CmdThrottle::SharedPtr);
    void cmd_rotation_sub(const ksp_bridge_interfaces::msg::CmdRotation::SharedPtr);

    // servers
    rclcpp::Service<ksp_bridge_interfaces::srv::Activation>::SharedPtr m_next_stage_srv;
    rclcpp::Service<ksp_bridge_interfaces::srv::SAS>::SharedPtr m_set_sas_srv;
    rclcpp::Service<ksp_bridge_interfaces::srv::String>::SharedPtr m_set_reference_frame_srv;

    void next_stage_srv(
        const ksp_bridge_interfaces::srv::Activation::Request::SharedPtr,
        const ksp_bridge_interfaces::srv::Activation::Response::SharedPtr);

    void set_sas_srv(
        const ksp_bridge_interfaces::srv::SAS::Request::SharedPtr,
        const ksp_bridge_interfaces::srv::SAS::Response::SharedPtr);

    void set_reference_frame(
        const ksp_bridge_interfaces::srv::String::Request::SharedPtr,
        const ksp_bridge_interfaces::srv::String::Response::SharedPtr);
};