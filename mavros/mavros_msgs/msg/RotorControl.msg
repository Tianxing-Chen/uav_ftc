#This message is sent to a low level controller on the quadrotor

std_msgs/Header header
# Control mode as defined above
uint8 control_mode

# Collective mass normalized thrust [m/s^2]
float64 thrust

# Single rotor throttle [0-2000]
float64[4] mot_throttle

# Single rotor thrust
float64[4] rotor_thrust
