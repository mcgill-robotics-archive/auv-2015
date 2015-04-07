uint8 ObjectID

# Keep these in sync with SlamTarget.msg
uint8 GATE=0
uint8 REDBUOY=1
uint8 YELLOWBUOY=2
uint8 GREENBUOY=3

float64 xPos
float64 yPos

# Covariance of x and y
float64 var_xx
float64 var_xy
float64 var_yy



