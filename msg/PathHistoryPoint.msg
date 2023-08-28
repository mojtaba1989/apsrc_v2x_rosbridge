float32 latOffset
## A range of +- 0.0131071 degrees
## The value +131071 shall be used for values >= than +0.0131071 degrees
## The value -131071 shall be used for values <= than -0.0131071 degrees
## The value -131072 shall be used unknown
## In LSB units of 0.1 microdegrees (unless a zoom is employed)

float32 longOffset
## A range of +- 0.0131071 degrees
## The value +131071 shall be used for values >= than +0.0131071 degrees
## The value -131071 shall be used for values <= than -0.0131071 degrees
## The value -131072 shall be used unknown
## In LSB units of 0.1 microdegrees (unless a zoom is employed)

float32 elevationOffset
# ASN.1 Representation:
# VertOffset-B12 ::= INTEGER (-2048..2047)
# -- LSB units of of 10 cm
# -- with a range of +- 204.7 meters vertical
# -- value 2047 to be used for 2047 or greater
# -- value -2047 to be used for -2047 or greater
# -- value -2048 to be unavailable

float32 timeOffset
# TimeOffset ::= INTEGER (1..65535)
# -- LSB units of of 10 mSec,
# -- with a range of 0.01 seconds to 10 minutes and 55.34 seconds
# -- a value of 65534 to be used for 655.34 seconds or greater
# -- a value of 65535 to be unavailable

