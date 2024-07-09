from dataclasses import dataclass


@dataclass
class ServoConfig:
	kp: float
	"""Proportional gain of the Servo controller"""

	kd: float
	"""Derivative gain of the Servo controller"""

	max_torque: float
	"""Maximum torque of the servo in N.m"""

	max_velocity: float
	"""Maximum velocity of the servo in rad/s"""

	lower_limit: float
	"""Lower limit of the servo in radians"""

	upper_limit: float
	"""Upper limit of the servo in radians"""