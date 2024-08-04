from simple_pid import PID

# Creating the PID controller with limits
THRESHOLD_DISTANCE = 100 # in cm
MAX_ALLOWABLE_SPEED_OF_VEHICLE = 20 # in cm/s


def read_distance_from_target():
	# some monocular camera distance estimation shit goes on here

def set_vehicle_speed():
	# link the sent speed to kinematics and use it to move. IMU can be used as feedback

pid = PID(0, 0, 0, setpoint=threshold_distance)
pid.sample_time = 0.1 # in seconds
pid.output_limits = (0, max_allowable_speed_of_vehicle)

# Tuning using Ziegler-Nichols Method.
Ku = 1 
Pu = 0.5 
pid.Kp = -(0.6 * Ku)
pid.Ki = -(2 * Kp/Pu)
pid.Kd = -(Kp * Pu/8)


set_vehicle_speed(0)
distance_from_target = read_distance_From_target()

while True:
	# Compute new output from the PID based on distance from threshold
	new_vehicle_speed = pid(distance)
	print(new_vehicle_speed)

	# Feed the PID output to the system and get distance
	set_vehicle_speed(new_vehicle_speed)
	distance_from_target = read_distance_from_target()
