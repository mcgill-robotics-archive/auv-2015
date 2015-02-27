# Don't forget to set the sensor frame in the header
Header header

# The name of the object, which will be the name of the tf frame published
# representing this object
string name

# Relative bearing
float64 bearing
float64 bearing_variance
# Slant range (along the line of sight)
float64 range
float64 range_variance
# If elevation is not available, set the covariance of elevation to be large
float64 elevation
float64 elevation_variance



