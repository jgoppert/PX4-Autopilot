uint32 id			# ID of the estimator, commonly the component ID of the incoming message

uint64 timestamp_boot		# time of this estimate, in microseconds since system start
uint64 timestamp_computer	# timestamp provided by the companion computer, in us

float32 x			# X velocity in meters per second in NED earth-fixed frame
float32 y			# Y velocity in meters per second in NED earth-fixed frame
float32 z			# Z velocity in meters per second in NED earth-fixed frame