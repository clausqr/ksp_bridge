# https://krpc.github.io/krpc/cpp/api/space-center/celestial-body.html

# Note: no satellites, biomes nor orbit

string name
float64 mass
float64 gravitational_parameter
float64 surface_gravity
float64 rotational_period
float64 rotational_speed
float64 rotation_angle
float64 initial_rotation
float64 equatorial_radius
float64 sphere_of_influence
bool has_atmosphere
float64 atmosphere_depth
bool has_atmospheric_oxygen
float32 flying_high_altitude_threshold
float32 space_high_altitude_threshold
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Quaternion rotation
geometry_msgs/Vector3 direction
geometry_msgs/Vector3 angular_velocity
