uint8 msgCnt
string ID
uint16 secMark
float64 Latitude #1/10 micro degree +-90
float64 Longitude #1/10 micro degree +-180
float32 Elevation #Providing a range of -409.5 to + 6143.9 meters
float32 PositionAccuracy # 0.05m
float32 OrientationAccuracy 
string TransmistionState #N0 P1 D2 R3
float32 Speed #Units of 0.02 m/s
float32 Heading #LSB of 0.0125 degrees
int8 SteeringWheelAngle #LSB units of 1.5 degrees 127->unavailable
AccelerationSet4Way AccelerationSet4Way
BrakeSystemStatus BrakeSystemStatus
VehicleSize VehicleSize