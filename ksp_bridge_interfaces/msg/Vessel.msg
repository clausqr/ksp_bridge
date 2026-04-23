# https://krpc.github.io/krpc/cpp/api/space-center/vessel.html

uint8 TYPE_BASE = 0
uint8 TYPE_DEBRIS = 1
uint8 TYPE_LANDER = 2
uint8 TYPE_PLANE = 3
uint8 TYPE_PROBE = 4
uint8 TYPE_RELAY = 5
uint8 TYPE_ROVER = 6
uint8 TYPE_SHIP = 7
uint8 TYPE_STATION = 8

uint8 SITUATION_PRE_LAUNCH = 0
uint8 SITUATION_ORBITING = 1
uint8 SITUATION_SUB_ORBITAL = 2
uint8 SITUATION_ESCAPING = 3
uint8 SITUATION_FLYING = 4
uint8 SITUATION_LANDED = 5
uint8 SITUATION_SPLASHED = 6
uint8 SITUATION_DOCKED = 7

std_msgs/Header header

string name
uint8 type
uint8 situation
bool recoverable
float64 met
string biome
int32 crew_capacity
int32 crew_count
float32 mass
float32 dry_mass
float32 thrust
float32 available_thrust
float32 max_thrust
float32 max_vacuum_thrust
float32 specific_impulse
float32 vacuum_specific_impulse
float32 kerbin_sea_level_specific_impulse
# Principal-axis diagonal in aerospace FRD body frame:
# x=roll (I_xx), y=pitch (I_yy), z=yaw (I_zz).
geometry_msgs/Vector3 moment_of_inertia
# Full inertia tensor in aerospace FRD body frame (from inertia_tensor,
# permuted from KSP vessel frame).
geometry_msgs/Inertia inertia
# position, velocity, direction, angular_velocity are in the active
# reference frame (kerbin celestial-body frame by default), which is
# KSP-native left-handed. FRD conversion only applies to body-frame
# quantities; see README "Frame conventions".
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
# Rotation from the active reference frame into the vessel FRD body
# frame. Permuted + w-flipped from kRPC's native quaternion.
geometry_msgs/Quaternion rotation
geometry_msgs/Vector3 direction
geometry_msgs/Vector3 angular_velocity
# Body-frame angular velocity (p, q, r) in aerospace FRD:
# x=roll rate, y=pitch rate, z=yaw rate. Computed server-side via
# transform_direction from body.non_rotating_reference_frame to
# vessel.reference_frame, then permuted to FRD.
geometry_msgs/Vector3 angular_velocity_body
