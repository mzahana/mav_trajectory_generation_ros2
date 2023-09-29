std_msgs/Header header
# order of the polynomial + 1, should match size of x[]
int32 num_coeffs
# duration of the segment
builtin_interfaces/Duration segment_time
# coefficients for the x-axis, INCREASING order
float64[] x
# coefficients for the y-axis, INCREASING order
float64[] y
# coefficients for the z-axis, INCREASING order
float64[] z
# coefficients for the yaw, INCREASING order
float64[] yaw