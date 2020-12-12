# Fused Reference States

# Reference States For Vehicle
float32 reference_surge_speed		# Surge Apeed (metre/second)
float32 reference_yaw_angle		# Yaw Angle (radian)
float32 way_point_x_position		# way-point x position (meter)
float32 way_point_y_position		# way-point y position (meter)
uint16 number_of_way_point              # Number of Defined Way Point
uint16 way_point_index                  # Current Way Point
bool finished				# Check Finished or Not

# Logged State Varibles (They must be logged here because of time synchronization)
float32 x				# North position in NED earth-fixed frame, (metres)
float32 y				# East position in NED earth-fixed frame, (metres)
float32 z				# Down position (negative altitude) in NED earth-fixed frame, (metres)

float32 yaw				# Heading
					# Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,(radians) 
float32[4] q				# Quaternion rotation from NED earth frame to XYZ body frame

float32 vx 				# North velocity in NED earth-fixed frame, (metres/sec)
float32 vy				# East velocity in NED earth-fixed frame, (metres/sec)
float32 vz				# Down velocity in NED earth-fixed frame, (metres/sec)

float32 rollspeed			# Bias corrected angular velocity about X body axis in rad/s
float32 pitchspeed			# Bias corrected angular velocity about Y body axis in rad/s
float32 yawspeed			# Bias corrected angular velocity about Z body axis in rad/s

# Force of the Thrusters
float32 left_thruster_force		# Left thruster force in Newton
float32 right_thruster_force 		# Right thruster force in Newton

# Cost Function Value
float32 output_error_cost		# Cost Related to Output Error Term
float32 control_effort_cost		# Cost Related to Control Effort Term
float32 total_cost			# Total Cost

# Wind Speed Information
float32 windspeed_north		# Wind component in north / X direction (m/sec)
float32 windspeed_east		# Wind component in east / Y direction (m/sec)

uint64 loop_time			# Loop Time for Controller
