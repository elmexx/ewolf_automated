# Topic Catalog

## /gnss/ecef

Message type: GnssEcef

Fields:

- header.stamp: ROS timestamp
- station: reference or correction station identifier
- pos.x/y/z: position in ECEF coordinates, expected unit metres
- velocity.x/y/z: velocity in ECEF coordinates, expected unit metres per second
- pos_error: estimated position error
- speed_error: estimated speed error

Usage:

- trajectory consistency checks
- position validity
- speed consistency
- localization quality feature extraction

## /gnss/ned

Message type: GnssNed

Fields:

- rel_pos_length: relative baseline length
- rel_pos_heading: relative heading
- rel_pos.x/y/z: relative position in NED coordinates
- rel_speed.x/y/z: relative speed in NED coordinates

Important:

The mapping of geometry_msgs/Vector3 fields to N/E/D must be confirmed from
the sensor driver documentation.

Do not assume that x, y and z always mean north, east and down without an
explicit configuration entry.

## /gnss/quality

Message type: GnssQuality

Contains estimated errors and DOP values:

- latitude_error
- longitude_error
- altitude_error
- speed_error
- pos2d_error
- pos3d_error
- PDOP
- HDOP
- VDOP
- TDOP
- GDOP

Usage:

- localization quality scoring
- invalid GNSS detection
- segment-level GNSS quality statistics

## /gnss/status

Message type: GnssStatus

Contains:

- sensor_time
- online
- status mask
- satellites_used
- satellites_visible
- DGPS station
- DGPS correction age

Important:

The meaning of individual mask bits is currently unknown.
Preserve the raw mask value.
Do not invent bit definitions.

## /imu/status

Message type: ImuStatus

Contains validity flags:

- accel_valid
- ypr_valid
- mag_valid
- gyro_valid

Usage:

- sensor availability
- feature validity masks
- detection of IMU degradation
