float32 latitude
float32 longitude
float32 elevation

# Position3D ::= SEQUENCE {
# lat
# Latitude,
# -- in 1/10th micro degrees
# long
# Longitude,
# -- in 1/10th micro degrees
# elevation Elevation OPTIONAL,
# -- in 10 cm units
# regional
# SEQUENCE (SIZE(1..4)) OF
# RegionalExtension {{REGION.Reg-Position3D}} OPTIONAL,
# ...
# }
