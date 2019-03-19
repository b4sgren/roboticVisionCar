# Vehicle state 'x_hat' output from the estimator
# Not sure we will use thisz

Header header

# States
float32 p_north # north position (m)
float32 p_east  # east position (m)
float32 psi     # unwrapped yaw angle (rad)
float32 b_r     # heading rate bias
float32 b_u     # velocity bias
float32 u       # body fixed forward velocity (m/s)

# Additional States for convenience
float32 psi_deg # Wrapped yaw angle (deg)
