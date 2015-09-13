# COLLISION_SENSOR message data
uint64 timestamp		# Microseconds since system boot.
uint16[16] collision_cm	# Distance in cm ultrasound measures.
uint8 sensor_count		# Number of sensors.
float32 covariance		# covariance of distance in m.