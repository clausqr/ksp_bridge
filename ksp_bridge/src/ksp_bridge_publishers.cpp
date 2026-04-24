#include <krpc/services/krpc.hpp>
#include <ksp_bridge/ksp_bridge.hpp>
#include <ksp_bridge/utils.hpp>
#include <ksp_bridge_interfaces/msg/celestial_body.hpp>
#include <ksp_bridge_interfaces/msg/resource.hpp>

void KSPBridge::publish_fast()
{
    if (!is_valid_screen()) {
        return;
    }

    validate_active_vessel();

    NamedReferenceFrame frame;
    m_refrence_frame.lock.lock();
    frame.name = m_refrence_frame.name;
    frame.refrence_frame = m_refrence_frame.refrence_frame;
    m_refrence_frame.lock.unlock();

    if (!m_fast_streams) {
        setup_fast_streams(frame);
        if (!m_fast_streams) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "fast-group streams unavailable; publish_fast ticks dropped");
            return;
        }
    }

    // Control fields are still on direct RPCs (WP5 scope). Gather them
    // BEFORE the frozen block so (a) they don't corrupt stream
    // frame-consistency and (b) if gather_control_data throws and
    // invalidates the vessel, we short-circuit before freezing.
    if (gather_control_data(frame)) {
        m_control_publisher->publish(m_control_data);
    }

    if (!m_fast_streams) {
        return;
    }

    // Pin stream values to one physics frame so fields read in the same
    // tick are mutually consistent. Safe because setup_fast_streams warmed
    // up every stream, so no operator() will need to start() and block on
    // the frozen update thread. tf_tree stays outside since it makes direct
    // RPCs.
    {
        struct ThawGuard {
            krpc::Client* client;
            ~ThawGuard() { if (client) client->thaw_streams(); }
        };
        m_ksp_client->freeze_streams();
        ThawGuard thaw_guard { m_ksp_client.get() };

        if (gather_vessel_data(frame)) {
            m_vessel_publisher->publish(m_vessel_data);
        }

        if (gather_flight_data(frame)) {
            m_flight_publisher->publish(m_flight_data);
        }

        if (gather_orbit_data()) {
            m_orbit_publisher->publish(m_orbit_data);
        }
    }

    send_tf_tree(frame);
}

void KSPBridge::publish_parts()
{
    if (!is_valid_screen()) {
        return;
    }

    validate_active_vessel();

    if (gather_parts_data()) {
        m_parts_publisher->publish(m_parts_data);
    }
}

void KSPBridge::publish_bodies()
{
    if (!is_valid_screen()) {
        return;
    }

    validate_active_vessel();

    NamedReferenceFrame frame;
    m_refrence_frame.lock.lock();
    frame.name = m_refrence_frame.name;
    frame.refrence_frame = m_refrence_frame.refrence_frame;
    m_refrence_frame.lock.unlock();

    if (gather_celestial_bodies_data(frame)) {
        m_celestial_bodies_publisher->publish(m_celestial_bodies_data);
    }
}

bool KSPBridge::gather_vessel_data(NamedReferenceFrame& frame)
{
    if (!m_vessel || !m_fast_streams) {
        return false;
    }
    try {
        auto& s = *m_fast_streams;

        m_vessel_data.header.frame_id = frame.name;
        m_vessel_data.header.stamp = now();
        m_vessel_data.name = s.vessel_name();
        m_vessel_data.type = (uint8_t)m_vessel->type();
        m_vessel_data.situation = (uint8_t)m_vessel->situation();
        m_vessel_data.recoverable = s.vessel_recoverable();

        m_vessel_data.met = s.vessel_met();
        m_vessel_data.biome = s.vessel_biome();
        m_vessel_data.crew_capacity = s.vessel_crew_capacity();
        m_vessel_data.crew_count = s.vessel_crew_count();
        float mass = s.vessel_mass();
        m_vessel_data.mass = mass;
        m_vessel_data.dry_mass = s.vessel_dry_mass();

        m_vessel_data.thrust = s.vessel_thrust();
        m_vessel_data.available_thrust = s.vessel_available_thrust();
        m_vessel_data.max_thrust = s.vessel_max_thrust();
        m_vessel_data.max_vacuum_thrust = s.vessel_max_vacuum_thrust();
        m_vessel_data.specific_impulse = s.vessel_specific_impulse();
        m_vessel_data.vacuum_specific_impulse = s.vessel_vacuum_specific_impulse();
        m_vessel_data.kerbin_sea_level_specific_impulse = s.vessel_kerbin_sea_level_specific_impulse();

        // moment_of_inertia is the principal-axis diagonal in the vessel
        // body frame → transform into FRD so I.x=roll, I.y=pitch, I.z=yaw.
        m_vessel_data.moment_of_inertia = vessel_frd_vector(tuple2vector3(s.vessel_moment_of_inertia()));

        auto position = s.vessel_position();
        auto inertia = s.vessel_inertia_tensor();

        m_vessel_data.inertia.m = mass;
        m_vessel_data.inertia.com = tuple2vector3(position);
        if (inertia.size() >= 6) {
            // inertia_tensor is a 3x3 symmetric tensor in the vessel body
            // frame, flattened to (ixx, ixy, ixz, iyy, iyz, izz). Under the
            // x<->y basis swap, diagonals (xx, yy) swap and cross terms
            // (xz, yz) swap; xy and zz are invariant.
            m_vessel_data.inertia.ixx = inertia[3];
            m_vessel_data.inertia.ixy = inertia[1];
            m_vessel_data.inertia.ixz = inertia[4];
            m_vessel_data.inertia.iyy = inertia[0];
            m_vessel_data.inertia.iyz = inertia[2];
            m_vessel_data.inertia.izz = inertia[5];
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "vessel_inertia_tensor returned %zu values; expected 6, leaving previous values",
                inertia.size());
        }

        // position, velocity, direction, angular_velocity are expressed in
        // the active reference frame (kerbin by default). They stay in
        // kRPC's native left-handed frame — FRD only applies to body-frame
        // quantities. See README "Frame conventions".
        m_vessel_data.position = tuple2vector3(position);
        m_vessel_data.velocity = tuple2vector3(s.vessel_velocity());
        m_vessel_data.rotation = vessel_frd_quaternion(tuple2quaternion(s.vessel_rotation()));
        m_vessel_data.direction = tuple2vector3(s.vessel_direction());
        m_vessel_data.angular_velocity = tuple2vector3(s.vessel_angular_velocity());
        // Body-frame ω via kRPC's canonical recipe: streamed ω in the SOI
        // body's inertial frame, then a single server-side transform into
        // the vessel frame. One sync RPC per tick, no cross-stream skew.
        m_vessel_data.angular_velocity_body = vessel_frd_pseudo_vector(tuple2vector3(
            m_space_center->transform_direction(
                s.vessel_angular_velocity_body_nonrot(),
                s.body_non_rotating_rf,
                s.vessel_rf)));
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
        return false;
    }

    return true;
}

bool KSPBridge::gather_control_data(NamedReferenceFrame& frame)
{
    if (!m_vessel) {
        return false;
    }
    try {
        auto control = m_vessel->control();

        m_control_data.header.frame_id = frame.name;
        m_control_data.header.stamp = now();
        m_control_data.source = (uint8_t)control.source();
        m_control_data.state = (uint8_t)control.state();
        m_control_data.sas = control.sas();
        m_control_data.sas_mode = (uint8_t)control.sas_mode();
        m_control_data.speed_mode = (uint8_t)control.speed_mode();
        m_control_data.rcs = control.rcs();
        m_control_data.reaction_wheels = control.reaction_wheels();
        m_control_data.gear = control.gear();
        m_control_data.legs = control.legs();
        m_control_data.wheels = control.wheels();
        m_control_data.lights = control.lights();
        m_control_data.brakes = control.brakes();
        m_control_data.antennas = control.antennas();
        m_control_data.cargo_bays = control.cargo_bays();
        m_control_data.intakes = control.intakes();
        m_control_data.parachutes = control.parachutes();
        m_control_data.radiators = control.radiators();
        m_control_data.resource_harvesters = control.resource_harvesters();
        m_control_data.resource_harvesters_active = control.resource_harvesters_active();
        m_control_data.solar_panels = control.solar_panels();
        m_control_data.abort = control.abort();
        m_control_data.throttle = control.throttle();
        m_control_data.input_mode = (uint8_t)control.input_mode();
        m_control_data.pitch = control.pitch();
        m_control_data.yaw = control.yaw();
        m_control_data.roll = control.roll();
        m_control_data.forward = control.forward();
        m_control_data.up = control.up();
        m_control_data.right = control.right();
        m_control_data.wheel_throttle = control.wheel_throttle();
        m_control_data.wheel_steering = control.wheel_steering();
        m_control_data.current_stage = control.current_stage();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
        return false;
    }

    return true;
}

bool KSPBridge::gather_flight_data(NamedReferenceFrame& frame)
{
    if (!m_vessel || !m_fast_streams) {
        return false;
    }
    try {
        auto& s = *m_fast_streams;

        m_flight_data.header.frame_id = frame.name;
        m_flight_data.header.stamp = now();

        m_flight_data.g_force = s.flight_g_force();
        m_flight_data.mean_altitude = s.flight_mean_altitude();
        m_flight_data.surface_altitude = s.flight_surface_altitude();
        m_flight_data.bedrock_altitude = s.flight_bedrock_altitude();
        m_flight_data.velocity = tuple2vector3(s.flight_velocity());
        m_flight_data.speed = s.flight_speed();
        m_flight_data.horizontal_speed = s.flight_horizontal_speed();
        m_flight_data.vertical_speed = s.flight_vertical_speed();
        m_flight_data.center_of_mass = tuple2vector3(s.flight_center_of_mass());
        m_flight_data.rotation = tuple2quaternion(s.flight_rotation());
        m_flight_data.direction = tuple2vector3(s.flight_direction());
        m_flight_data.pitch = s.flight_pitch();
        m_flight_data.heading = s.flight_heading();
        m_flight_data.roll = s.flight_roll();
        m_flight_data.prograde = tuple2vector3(s.flight_prograde());
        m_flight_data.retrograde = tuple2vector3(s.flight_retrograde());
        m_flight_data.normal = tuple2vector3(s.flight_normal());
        m_flight_data.anti_normal = tuple2vector3(s.flight_anti_normal());
        m_flight_data.radial = tuple2vector3(s.flight_radial());
        m_flight_data.anti_radial = tuple2vector3(s.flight_anti_radial());
        m_flight_data.atmosphere_density = s.flight_atmosphere_density();
        m_flight_data.dynamic_pressure = s.flight_dynamic_pressure();
        m_flight_data.static_pressure = s.flight_static_pressure();
        m_flight_data.static_pressure_at_msl = s.flight_static_pressure_at_msl();
        m_flight_data.aerodynamic_force = tuple2vector3(s.flight_aerodynamic_force());
        m_flight_data.lift = tuple2vector3(s.flight_lift());
        m_flight_data.drag = tuple2vector3(s.flight_drag());
        m_flight_data.speed_of_sound = s.flight_speed_of_sound();
        m_flight_data.mach = s.flight_mach();
        m_flight_data.true_air_speed = s.flight_true_air_speed();
        m_flight_data.equivalent_air_speed = s.flight_equivalent_air_speed();
        m_flight_data.terminal_velocity = s.flight_terminal_velocity();
        m_flight_data.angle_of_attack = s.flight_angle_of_attack();
        m_flight_data.sideslip_angle = s.flight_sideslip_angle();
        m_flight_data.total_air_temperature = s.flight_total_air_temperature();
        m_flight_data.static_air_temperature = s.flight_static_air_temperature();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
        return false;
    }

    return true;
}

bool KSPBridge::gather_parts_data()
{
    if (!m_vessel) {
        return false;
    }
    try {
        auto parts = m_vessel->parts().all();

        // FIXME: do not reallocate
        m_parts_data.parts.clear();

        m_parts_data.header.frame_id = "vessel";
        m_parts_data.header.stamp = now();
        auto vessel_rf = m_vessel->reference_frame();

        for (auto& part : parts) {
            try {
                auto part_data = ksp_bridge_interfaces::msg::Part();

                int part_count = std::count_if(m_parts_data.parts.begin(), m_parts_data.parts.end(),
                    [&](const ksp_bridge_interfaces::msg::Part& p) {
                        return p.title == part.title();
                    });

                part_data.name = part.name();
                part_data.title = part.title();
                part_data.tag = "#" + std::to_string(part_count);
                part_data.highlighted = part.highlighted();
                part_data.highlight_color = tuple2vector3(part.highlight_color());
                part_data.cost = part.cost();
                part_data.axially_attached = part.axially_attached();
                part_data.radially_attached = part.radially_attached();
                part_data.stage = part.stage();
                part_data.decouple_stage = part.decouple_stage();
                part_data.massless = part.massless();
                part_data.mass = part.mass();
                part_data.dry_mass = part.dry_mass();
                part_data.shielded = part.shielded();
                part_data.dynamic_pressure = part.dynamic_pressure();
                part_data.impact_tolerance = part.impact_tolerance();
                part_data.temperature = part.temperature();
                part_data.skin_temperature = part.skin_temperature();
                part_data.max_temperature = part.max_temperature();
                part_data.max_skin_temperature = part.max_skin_temperature();
                part_data.thermal_mass = part.thermal_mass();
                part_data.thermal_skin_mass = part.thermal_skin_mass();
                part_data.thermal_resource_mass = part.thermal_resource_mass();
                part_data.thermal_conduction_flux = part.thermal_conduction_flux();
                part_data.thermal_convection_flux = part.thermal_convection_flux();
                part_data.thermal_radiation_flux = part.thermal_radiation_flux();
                part_data.thermal_internal_flux = part.thermal_internal_flux();
                part_data.thermal_skin_to_internal_flux = part.thermal_skin_to_internal_flux();

                auto resources = part.resources().all();
                for (auto& resource : resources) {
                    ksp_bridge_interfaces::msg::Resource r;

                    r.name = resource.name();
                    r.amount = resource.amount();
                    r.max = resource.max();
                    r.density = resource.density();
                    r.flow_mode = (uint8_t)resource.flow_mode();
                    r.enabled = resource.enabled();

                    part_data.resources.emplace_back(r);
                }

                part_data.crossfeed = part.crossfeed();
                part_data.is_fuel_line = part.is_fuel_line();
                part_data.position = tuple2vector3(part.position(vessel_rf));
                part_data.center_of_mass = tuple2vector3(part.center_of_mass(vessel_rf));
                part_data.direction = tuple2vector3(part.direction(vessel_rf));
                part_data.velocity = tuple2vector3(part.velocity(vessel_rf));
                part_data.rotation = tuple2quaternion(part.rotation(vessel_rf));
                part_data.moment_of_inertia = tuple2vector3(part.moment_of_inertia());

                part_data.inertia.m = part.mass();
                part_data.inertia.com = tuple2vector3(part.position(vessel_rf));
                // TODO: check this
                part_data.inertia.ixx = part.inertia_tensor()[0];
                part_data.inertia.ixy = part.inertia_tensor()[1];
                part_data.inertia.ixz = part.inertia_tensor()[2];
                part_data.inertia.iyy = part.inertia_tensor()[3];
                part_data.inertia.iyz = part.inertia_tensor()[4];
                part_data.inertia.izz = part.inertia_tensor()[5];

                m_parts_data.parts.emplace_back(part_data);
            } catch (const std::exception& ex) {
                RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
                continue;
            }
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
        return false;
    }

    return true;
}

bool KSPBridge::gather_celestial_bodies_data(NamedReferenceFrame& frame)
{
    try {
        // FIXME: do not reallocate
        m_celestial_bodies_data.bodies.clear();

        m_celestial_bodies_data.header.frame_id = frame.name;
        m_celestial_bodies_data.header.stamp = now();

        for (auto it = m_celestial_bodies.begin(); it != m_celestial_bodies.end(); ++it) {
            auto body = it->second;
            ksp_bridge_interfaces::msg::CelestialBody body_data;

            body_data.name = body.name();
            body_data.mass = body.mass();
            body_data.gravitational_parameter = body.gravitational_parameter();
            body_data.surface_gravity = body.surface_gravity();
            body_data.rotational_period = body.rotational_period();
            body_data.rotational_speed = body.rotational_speed();
            body_data.rotation_angle = body.rotation_angle();
            body_data.initial_rotation = body.initial_rotation();
            body_data.equatorial_radius = body.equatorial_radius();
            body_data.sphere_of_influence = body.sphere_of_influence();
            body_data.has_atmosphere = body.has_atmosphere();
            body_data.atmosphere_depth = body.atmosphere_depth();
            body_data.has_atmospheric_oxygen = body.has_atmospheric_oxygen();
            body_data.flying_high_altitude_threshold = body.flying_high_altitude_threshold();
            body_data.space_high_altitude_threshold = body.space_high_altitude_threshold();

            body_data.position = tuple2vector3(body.position(frame.refrence_frame));
            body_data.velocity = tuple2vector3(body.velocity(frame.refrence_frame));
            body_data.rotation = tuple2quaternion(body.rotation(frame.refrence_frame));
            body_data.direction = tuple2vector3(body.direction(frame.refrence_frame));
            body_data.angular_velocity = tuple2vector3(body.angular_velocity(frame.refrence_frame));

            m_celestial_bodies_data.bodies.emplace_back(body_data);
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
        return false;
    }

    return true;
}

bool KSPBridge::gather_orbit_data()
{
    if (!m_vessel || !m_fast_streams) {
        return false;
    }
    try {
        auto& s = *m_fast_streams;

        // Detect SOI transition by comparing streamed body handle to the
        // cached one; only then pay one direct RPC for the new name.
        auto current_body = s.orbit_body();
        if (!(current_body == s.orbit_body_cached)) {
            s.orbit_body_cached = current_body;
            s.orbit_body_name = current_body.name();
        }
        m_orbit_data.body = s.orbit_body_name;
        m_orbit_data.apoapsis = s.orbit_apoapsis();
        m_orbit_data.periapsis = s.orbit_periapsis();
        m_orbit_data.apoapsis_altitude = s.orbit_apoapsis_altitude();
        m_orbit_data.periapsis_altitude = s.orbit_periapsis_altitude();
        m_orbit_data.semi_major_axis = s.orbit_semi_major_axis();
        m_orbit_data.semi_minor_axis = s.orbit_semi_minor_axis();
        m_orbit_data.radius = s.orbit_radius();
        m_orbit_data.speed = s.orbit_speed();
        m_orbit_data.period = s.orbit_period();
        m_orbit_data.time_to_apoapsis = s.orbit_time_to_apoapsis();
        m_orbit_data.time_to_periapsis = s.orbit_time_to_periapsis();
        m_orbit_data.eccentricity = s.orbit_eccentricity();
        m_orbit_data.inclination = s.orbit_inclination();
        m_orbit_data.longitude_of_ascending_node = s.orbit_longitude_of_ascending_node();
        m_orbit_data.argument_of_periapsis = s.orbit_argument_of_periapsis();
        m_orbit_data.mean_anomaly_at_epoch = s.orbit_mean_anomaly_at_epoch();
        m_orbit_data.epoch = s.orbit_epoch();
        m_orbit_data.mean_anomaly = s.orbit_mean_anomaly();
        m_orbit_data.eccentric_anomaly = s.orbit_eccentric_anomaly();
        m_orbit_data.true_anomaly = s.orbit_true_anomaly();
        m_orbit_data.orbital_speed = s.orbit_orbital_speed();
        m_orbit_data.time_to_soi_change = s.orbit_time_to_soi_change();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR(get_logger(), "%s:%d: %s", base_name(__FILE__), __LINE__, ex.what());
        invalidate_active_vessel();
        return false;
    }

    return true;
}