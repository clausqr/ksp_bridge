#include <chrono>
#include <stdexcept>

#include <krpc/services/krpc.hpp>
#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>

KSPBridge::KSPBridge()
    : rclcpp::Node("ksp_bridge")
{
    declare_parameter<double>("publish_rate_hz", 10.0);
    declare_parameter<double>("parts_publish_rate_hz", 1.0);
    declare_parameter<double>("bodies_publish_rate_hz", 1.0);
    declare_parameter<std::vector<std::string>>("celestial_bodies", {"kerbin"});

    double fast_rate_hz = get_parameter("publish_rate_hz").as_double();
    double parts_rate_hz = get_parameter("parts_publish_rate_hz").as_double();
    double bodies_rate_hz = get_parameter("bodies_publish_rate_hz").as_double();
    m_param_celestial_bodies = get_parameter("celestial_bodies").as_string_array();

    auto validated_period = [this](const char* name, double rate_hz) {
        if (rate_hz <= 0.0) {
            RCLCPP_FATAL(get_logger(), "%s must be > 0, got %f", name, rate_hz);
            throw std::invalid_argument(std::string(name) + " must be > 0");
        }
        if (rate_hz > 50.0) {
            RCLCPP_WARN_ONCE(get_logger(),
                "%s=%f exceeds kRPC's ~50 Hz physics-tick ceiling; "
                "values above the in-game Physics.fixedDeltaTime rate will yield duplicate "
                "samples from the same physics frame. See the kRPC server window "
                "(\"Max time per update\" and \"Blocking receives\") if this is intentional.",
                name, rate_hz);
        }
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate_hz));
    };

    auto fast_period = validated_period("publish_rate_hz", fast_rate_hz);
    auto parts_period = validated_period("parts_publish_rate_hz", parts_rate_hz);
    auto bodies_period = validated_period("bodies_publish_rate_hz", bodies_rate_hz);

    connect();
    find_active_vessel();

    m_fast_timer = create_wall_timer(fast_period, std::bind(&KSPBridge::publish_fast, this));
    m_parts_timer = create_wall_timer(parts_period, std::bind(&KSPBridge::publish_parts, this));
    m_bodies_timer = create_wall_timer(bodies_period, std::bind(&KSPBridge::publish_bodies, this));
}

void KSPBridge::connect()
{
    while (rclcpp::ok()) {
        try {
            m_ksp_client = std::make_unique<krpc::Client>(krpc::connect("ksp_bridge"));

            m_space_center = std::make_unique<krpc::services::SpaceCenter>(m_ksp_client.get());
            m_krpc = std::make_unique<krpc::services::KRPC>(m_ksp_client.get());
            break;
        } catch (...) {
            RCLCPP_INFO(get_logger(), "Connecting to kRPC server ...");
        }

        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    RCLCPP_INFO(get_logger(), "Connected to kRPC server v%s", m_krpc->get_status().version().c_str());
}

bool KSPBridge::is_valid_screen()
{
    auto scene = m_krpc->current_game_scene();

    if (scene == krpc::services::KRPC::GameScene::flight) {
        return true;
    } else {
        RCLCPP_WARN(get_logger(), "Invalid game screen.");
        invalidate_active_vessel();
        return false;
    }
}

void KSPBridge::validate_active_vessel()
{
    // vessel has been invalidated
    if (m_vessel == nullptr) {
        connect();
        find_active_vessel();
        return;
    }

    bool is_valid = false;
    try {
        m_vessel = std::make_unique<krpc::services::SpaceCenter::Vessel>(m_space_center->active_vessel());
        is_valid = true;
    } catch (...) {
    }

    if (!is_valid) {
        find_active_vessel();
    }
}

void KSPBridge::invalidate_active_vessel()
{
    m_vessel = nullptr;
    teardown_fast_streams();
    RCLCPP_WARN(get_logger(), "Vessel has been invalidated.");
}

void KSPBridge::teardown_fast_streams()
{
    if (!m_fast_streams) {
        return;
    }
    auto& b = *m_fast_streams;
    auto safe_remove = [](auto& stream) {
        try {
            stream.remove();
        } catch (...) {
            // Swallow: we're tearing down anyway, and the server-side
            // stream may already be gone if the vessel was destroyed.
        }
    };

    safe_remove(b.vessel_name);
    safe_remove(b.vessel_type);
    safe_remove(b.vessel_situation);
    safe_remove(b.vessel_recoverable);
    safe_remove(b.vessel_met);
    safe_remove(b.vessel_biome);
    safe_remove(b.vessel_crew_capacity);
    safe_remove(b.vessel_crew_count);
    safe_remove(b.vessel_mass);
    safe_remove(b.vessel_dry_mass);
    safe_remove(b.vessel_thrust);
    safe_remove(b.vessel_available_thrust);
    safe_remove(b.vessel_max_thrust);
    safe_remove(b.vessel_max_vacuum_thrust);
    safe_remove(b.vessel_specific_impulse);
    safe_remove(b.vessel_vacuum_specific_impulse);
    safe_remove(b.vessel_kerbin_sea_level_specific_impulse);
    safe_remove(b.vessel_moment_of_inertia);
    safe_remove(b.vessel_inertia_tensor);
    safe_remove(b.vessel_position);
    safe_remove(b.vessel_velocity);
    safe_remove(b.vessel_rotation);
    safe_remove(b.vessel_direction);
    safe_remove(b.vessel_angular_velocity);

    safe_remove(b.flight_g_force);
    safe_remove(b.flight_mean_altitude);
    safe_remove(b.flight_surface_altitude);
    safe_remove(b.flight_bedrock_altitude);
    safe_remove(b.flight_velocity);
    safe_remove(b.flight_speed);
    safe_remove(b.flight_horizontal_speed);
    safe_remove(b.flight_vertical_speed);
    safe_remove(b.flight_center_of_mass);
    safe_remove(b.flight_rotation);
    safe_remove(b.flight_direction);
    safe_remove(b.flight_pitch);
    safe_remove(b.flight_heading);
    safe_remove(b.flight_roll);
    safe_remove(b.flight_prograde);
    safe_remove(b.flight_retrograde);
    safe_remove(b.flight_normal);
    safe_remove(b.flight_anti_normal);
    safe_remove(b.flight_radial);
    safe_remove(b.flight_anti_radial);
    safe_remove(b.flight_atmosphere_density);
    safe_remove(b.flight_dynamic_pressure);
    safe_remove(b.flight_static_pressure);
    safe_remove(b.flight_static_pressure_at_msl);
    safe_remove(b.flight_aerodynamic_force);
    safe_remove(b.flight_lift);
    safe_remove(b.flight_drag);
    safe_remove(b.flight_speed_of_sound);
    safe_remove(b.flight_mach);
    safe_remove(b.flight_true_air_speed);
    safe_remove(b.flight_equivalent_air_speed);
    safe_remove(b.flight_terminal_velocity);
    safe_remove(b.flight_angle_of_attack);
    safe_remove(b.flight_sideslip_angle);
    safe_remove(b.flight_total_air_temperature);
    safe_remove(b.flight_static_air_temperature);

    safe_remove(b.orbit_apoapsis);
    safe_remove(b.orbit_periapsis);
    safe_remove(b.orbit_apoapsis_altitude);
    safe_remove(b.orbit_periapsis_altitude);
    safe_remove(b.orbit_semi_major_axis);
    safe_remove(b.orbit_semi_minor_axis);
    safe_remove(b.orbit_radius);
    safe_remove(b.orbit_speed);
    safe_remove(b.orbit_period);
    safe_remove(b.orbit_time_to_apoapsis);
    safe_remove(b.orbit_time_to_periapsis);
    safe_remove(b.orbit_eccentricity);
    safe_remove(b.orbit_inclination);
    safe_remove(b.orbit_longitude_of_ascending_node);
    safe_remove(b.orbit_argument_of_periapsis);
    safe_remove(b.orbit_mean_anomaly_at_epoch);
    safe_remove(b.orbit_epoch);
    safe_remove(b.orbit_mean_anomaly);
    safe_remove(b.orbit_eccentric_anomaly);
    safe_remove(b.orbit_true_anomaly);
    safe_remove(b.orbit_orbital_speed);
    safe_remove(b.orbit_time_to_soi_change);
    safe_remove(b.orbit_body);

    m_fast_streams.reset();
}

void KSPBridge::setup_fast_streams(NamedReferenceFrame& frame)
{
    if (!m_vessel) {
        return;
    }

    auto b = std::make_unique<FastStreams>();
    try {
        auto flight = m_vessel->flight(frame.refrence_frame);
        auto orbit = m_vessel->orbit();

        b->orbit_body = orbit.body_stream();
        b->orbit_body_cached = orbit.body();
        b->orbit_body_name = b->orbit_body_cached.name();

        b->vessel_name = m_vessel->name_stream();
        b->vessel_type = m_vessel->type_stream();
        b->vessel_situation = m_vessel->situation_stream();
        b->vessel_recoverable = m_vessel->recoverable_stream();
        b->vessel_met = m_vessel->met_stream();
        b->vessel_biome = m_vessel->biome_stream();
        b->vessel_crew_capacity = m_vessel->crew_capacity_stream();
        b->vessel_crew_count = m_vessel->crew_count_stream();
        b->vessel_mass = m_vessel->mass_stream();
        b->vessel_dry_mass = m_vessel->dry_mass_stream();
        b->vessel_thrust = m_vessel->thrust_stream();
        b->vessel_available_thrust = m_vessel->available_thrust_stream();
        b->vessel_max_thrust = m_vessel->max_thrust_stream();
        b->vessel_max_vacuum_thrust = m_vessel->max_vacuum_thrust_stream();
        b->vessel_specific_impulse = m_vessel->specific_impulse_stream();
        b->vessel_vacuum_specific_impulse = m_vessel->vacuum_specific_impulse_stream();
        b->vessel_kerbin_sea_level_specific_impulse = m_vessel->kerbin_sea_level_specific_impulse_stream();
        b->vessel_moment_of_inertia = m_vessel->moment_of_inertia_stream();
        b->vessel_inertia_tensor = m_vessel->inertia_tensor_stream();

        b->vessel_position = m_vessel->position_stream(frame.refrence_frame);
        b->vessel_velocity = m_vessel->velocity_stream(frame.refrence_frame);
        b->vessel_rotation = m_vessel->rotation_stream(frame.refrence_frame);
        b->vessel_direction = m_vessel->direction_stream(frame.refrence_frame);
        b->vessel_angular_velocity = m_vessel->angular_velocity_stream(frame.refrence_frame);

        b->flight_g_force = flight.g_force_stream();
        b->flight_mean_altitude = flight.mean_altitude_stream();
        b->flight_surface_altitude = flight.surface_altitude_stream();
        b->flight_bedrock_altitude = flight.bedrock_altitude_stream();
        b->flight_velocity = flight.velocity_stream();
        b->flight_speed = flight.speed_stream();
        b->flight_horizontal_speed = flight.horizontal_speed_stream();
        b->flight_vertical_speed = flight.vertical_speed_stream();
        b->flight_center_of_mass = flight.center_of_mass_stream();
        b->flight_rotation = flight.rotation_stream();
        b->flight_direction = flight.direction_stream();
        b->flight_pitch = flight.pitch_stream();
        b->flight_heading = flight.heading_stream();
        b->flight_roll = flight.roll_stream();
        b->flight_prograde = flight.prograde_stream();
        b->flight_retrograde = flight.retrograde_stream();
        b->flight_normal = flight.normal_stream();
        b->flight_anti_normal = flight.anti_normal_stream();
        b->flight_radial = flight.radial_stream();
        b->flight_anti_radial = flight.anti_radial_stream();
        b->flight_atmosphere_density = flight.atmosphere_density_stream();
        b->flight_dynamic_pressure = flight.dynamic_pressure_stream();
        b->flight_static_pressure = flight.static_pressure_stream();
        b->flight_static_pressure_at_msl = flight.static_pressure_at_msl_stream();
        b->flight_aerodynamic_force = flight.aerodynamic_force_stream();
        b->flight_lift = flight.lift_stream();
        b->flight_drag = flight.drag_stream();
        b->flight_speed_of_sound = flight.speed_of_sound_stream();
        b->flight_mach = flight.mach_stream();
        b->flight_true_air_speed = flight.true_air_speed_stream();
        b->flight_equivalent_air_speed = flight.equivalent_air_speed_stream();
        b->flight_terminal_velocity = flight.terminal_velocity_stream();
        b->flight_angle_of_attack = flight.angle_of_attack_stream();
        b->flight_sideslip_angle = flight.sideslip_angle_stream();
        b->flight_total_air_temperature = flight.total_air_temperature_stream();
        b->flight_static_air_temperature = flight.static_air_temperature_stream();

        b->orbit_apoapsis = orbit.apoapsis_stream();
        b->orbit_periapsis = orbit.periapsis_stream();
        b->orbit_apoapsis_altitude = orbit.apoapsis_altitude_stream();
        b->orbit_periapsis_altitude = orbit.periapsis_altitude_stream();
        b->orbit_semi_major_axis = orbit.semi_major_axis_stream();
        b->orbit_semi_minor_axis = orbit.semi_minor_axis_stream();
        b->orbit_radius = orbit.radius_stream();
        b->orbit_speed = orbit.speed_stream();
        b->orbit_period = orbit.period_stream();
        b->orbit_time_to_apoapsis = orbit.time_to_apoapsis_stream();
        b->orbit_time_to_periapsis = orbit.time_to_periapsis_stream();
        b->orbit_eccentricity = orbit.eccentricity_stream();
        b->orbit_inclination = orbit.inclination_stream();
        b->orbit_longitude_of_ascending_node = orbit.longitude_of_ascending_node_stream();
        b->orbit_argument_of_periapsis = orbit.argument_of_periapsis_stream();
        b->orbit_mean_anomaly_at_epoch = orbit.mean_anomaly_at_epoch_stream();
        b->orbit_epoch = orbit.epoch_stream();
        b->orbit_mean_anomaly = orbit.mean_anomaly_stream();
        b->orbit_eccentric_anomaly = orbit.eccentric_anomaly_stream();
        b->orbit_true_anomaly = orbit.true_anomaly_stream();
        b->orbit_orbital_speed = orbit.orbital_speed_stream();
        b->orbit_time_to_soi_change = orbit.time_to_soi_change_stream();

        // Warmup: force each stream to start() and receive its first value.
        // operator() lazily calls start() and waits on the update thread's
        // condition variable; doing this now (outside any freeze_streams
        // block) guarantees subsequent reads inside freeze/thaw won't
        // deadlock on an unstarted stream.
        (void)b->vessel_name();
        (void)b->vessel_type();
        (void)b->vessel_situation();
        (void)b->vessel_recoverable();
        (void)b->vessel_met();
        (void)b->vessel_biome();
        (void)b->vessel_crew_capacity();
        (void)b->vessel_crew_count();
        (void)b->vessel_mass();
        (void)b->vessel_dry_mass();
        (void)b->vessel_thrust();
        (void)b->vessel_available_thrust();
        (void)b->vessel_max_thrust();
        (void)b->vessel_max_vacuum_thrust();
        (void)b->vessel_specific_impulse();
        (void)b->vessel_vacuum_specific_impulse();
        (void)b->vessel_kerbin_sea_level_specific_impulse();
        (void)b->vessel_moment_of_inertia();
        (void)b->vessel_inertia_tensor();
        (void)b->vessel_position();
        (void)b->vessel_velocity();
        (void)b->vessel_rotation();
        (void)b->vessel_direction();
        (void)b->vessel_angular_velocity();
        (void)b->flight_g_force();
        (void)b->flight_mean_altitude();
        (void)b->flight_surface_altitude();
        (void)b->flight_bedrock_altitude();
        (void)b->flight_velocity();
        (void)b->flight_speed();
        (void)b->flight_horizontal_speed();
        (void)b->flight_vertical_speed();
        (void)b->flight_center_of_mass();
        (void)b->flight_rotation();
        (void)b->flight_direction();
        (void)b->flight_pitch();
        (void)b->flight_heading();
        (void)b->flight_roll();
        (void)b->flight_prograde();
        (void)b->flight_retrograde();
        (void)b->flight_normal();
        (void)b->flight_anti_normal();
        (void)b->flight_radial();
        (void)b->flight_anti_radial();
        (void)b->flight_atmosphere_density();
        (void)b->flight_dynamic_pressure();
        (void)b->flight_static_pressure();
        (void)b->flight_static_pressure_at_msl();
        (void)b->flight_aerodynamic_force();
        (void)b->flight_lift();
        (void)b->flight_drag();
        (void)b->flight_speed_of_sound();
        (void)b->flight_mach();
        (void)b->flight_true_air_speed();
        (void)b->flight_equivalent_air_speed();
        (void)b->flight_terminal_velocity();
        (void)b->flight_angle_of_attack();
        (void)b->flight_sideslip_angle();
        (void)b->flight_total_air_temperature();
        (void)b->flight_static_air_temperature();
        (void)b->orbit_apoapsis();
        (void)b->orbit_periapsis();
        (void)b->orbit_apoapsis_altitude();
        (void)b->orbit_periapsis_altitude();
        (void)b->orbit_semi_major_axis();
        (void)b->orbit_semi_minor_axis();
        (void)b->orbit_radius();
        (void)b->orbit_speed();
        (void)b->orbit_period();
        (void)b->orbit_time_to_apoapsis();
        (void)b->orbit_time_to_periapsis();
        (void)b->orbit_eccentricity();
        (void)b->orbit_inclination();
        (void)b->orbit_longitude_of_ascending_node();
        (void)b->orbit_argument_of_periapsis();
        (void)b->orbit_mean_anomaly_at_epoch();
        (void)b->orbit_epoch();
        (void)b->orbit_mean_anomaly();
        (void)b->orbit_eccentric_anomaly();
        (void)b->orbit_true_anomaly();
        (void)b->orbit_orbital_speed();
        (void)b->orbit_time_to_soi_change();
        (void)b->orbit_body();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "setup_fast_streams failed: %s", ex.what());
        return;
    }

    m_fast_streams = std::move(b);
    RCLCPP_INFO(get_logger(), "Fast-group kRPC streams registered and warmed up.");
}

void KSPBridge::find_active_vessel()
{
    while (rclcpp::ok()) {
        try {
            m_vessel = std::make_unique<krpc::services::SpaceCenter::Vessel>(m_space_center->active_vessel());
            m_vessel->control().set_input_mode(krpc::services::SpaceCenter::ControlInputMode::override);
            RCLCPP_INFO(get_logger(), "Vessel found: '%s'", m_vessel->name().c_str());
            break;
        } catch (...) {
            m_vessel = nullptr;
            RCLCPP_INFO(get_logger(), "Searching active vessel ...");
        }
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    init_celestial_bodies();
    init_interfaces();
}

void KSPBridge::init_celestial_bodies()
{
    auto bodies = m_space_center->bodies();
    m_celestial_bodies.clear();

    for (auto it = bodies.begin(); it != bodies.end(); ++it) {
        auto name = it->second.name();
        auto name_lower = str_lowercase(name);

        auto it_find = std::find(m_param_celestial_bodies.begin(), m_param_celestial_bodies.end(), name_lower);

        if (it_find != m_param_celestial_bodies.end()) {
            m_celestial_bodies[name_lower] = it->second;
        }
    }

    auto it = m_celestial_bodies.find("kerbin");
    if (it == m_celestial_bodies.end()) {
        m_celestial_bodies["kerbin"] = bodies["Kerbin"];
        RCLCPP_WARN(get_logger(), "'kerbin' is added to the celestial bodies.");
    }

    if (!change_reference_frame("kerbin")) {
        RCLCPP_ERROR(get_logger(), "Unable to change the reference frame to 'kerbin'.");
    }
}

void KSPBridge::init_interfaces()
{
    m_vessel_publisher = create_publisher<ksp_bridge_interfaces::msg::Vessel>("/vessel", 10);
    m_control_publisher = create_publisher<ksp_bridge_interfaces::msg::Control>("/vessel/control", 10);
    m_flight_publisher = create_publisher<ksp_bridge_interfaces::msg::Flight>("/vessel/flight", 10);
    m_parts_publisher = create_publisher<ksp_bridge_interfaces::msg::Parts>("/vessel/parts", 10);
    m_celestial_bodies_publisher = create_publisher<ksp_bridge_interfaces::msg::CelestialBodies>("/celestial_bodies", 10);
    m_orbit_publisher = create_publisher<ksp_bridge_interfaces::msg::Orbit>("/vessel/orbit", 10);

    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    m_cmd_throttle_sub = create_subscription<ksp_bridge_interfaces::msg::CmdThrottle>(
        "/cmd_throttle",
        10,
        std::bind(&KSPBridge::cmd_throttle_sub, this, std::placeholders::_1));

    m_cmd_rotation_sub = create_subscription<ksp_bridge_interfaces::msg::CmdRotation>(
        "/cmd_rotation",
        10,
        std::bind(&KSPBridge::cmd_rotation_sub, this, std::placeholders::_1));

    m_next_stage_srv = create_service<ksp_bridge_interfaces::srv::Activation>(
        "/next_stage",
        std::bind(
            &KSPBridge::next_stage_srv, this,
            std::placeholders::_1, std::placeholders::_2));

    m_set_sas_srv = create_service<ksp_bridge_interfaces::srv::SAS>(
        "/set_sas",
        std::bind(
            &KSPBridge::set_sas_srv, this,
            std::placeholders::_1, std::placeholders::_2));

    m_set_reference_frame_srv = create_service<ksp_bridge_interfaces::srv::String>(
        "/set_reference_frame",
        std::bind(
            &KSPBridge::set_reference_frame, this,
            std::placeholders::_1, std::placeholders::_2));
}
