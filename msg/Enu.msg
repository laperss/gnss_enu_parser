# Navigation ENU for any Global Navigation Satellite System

# header.stamp specifies the ROS time for this measurement (the
#        corresponding satellite time may be reported using the
#        sensor_msgs/TimeReference message).
#
# header.frame_id is the frame of reference reported by the satellite
#        receiver, usually the location of the antenna.  This is a
#        Euclidean frame relative to the vehicle, not a reference
#        ellipsoid.
Header header

# satellite fix status information
int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

uint8 status

# East [m]. Distance from reference station to the east.
float32 east

# North [m]. Distance from reference station to the north.
float32 north

# Up [m]. Distance from reference station up
float32 up

# Position covariance [m^2]. The components are East, North, and
# Up (ENU), in row-major order.

float32[9] covariance

# The number of satellites used in this solution
uint8 numsat

# Ambiguity ratio
float32 ratio

# Age of differential
float32 age

