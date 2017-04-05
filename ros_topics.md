# Namespaces and Topics
### where they go, what they do

#### General boat topics:
* `location` The boat's location relative to startup, in meters with North at +y and
  * Pose2D (pub from simulator for now) (x and y orthogonal) (angle Range 0-359.999)
* `velocity` The boat's smoothed current velocity, relative to the world
  * Pose2D (x and y orthogonal) (angle Range 0-359.999)

#### Waypoints:
* `new_waypoint` input topic for new waypoints (Pose2D)
* `clear_waypoints` True clears waypoint list (Bool)
* `rm_waypoint` True removes most recently added waypoint (Bool)
* `reached_waypoint` True removes the current target waypoint (Bool)
* `next_waypoint` Updated with the next waypoint in the list (Pose2D)
* `has_waypoints` True if further waypoints exist, False otherwise (Bool)


#### Intermediate values:
* `next_waypoint_rel` Position of the next waypoint relative to the boat, with x being a distance in meters and theta being an angle -180<theta<=180 (Pose2D)
* `cmd_vel` Commanded velocity of the boat (geometry_msgs/Twist)

#### Sensor values:
* `imu/data` (type sensor_msgs/Imu) Raw data retrieved from primary IMU sensor
* `temperature` (type sensor_msgs/Temperature) Temperature in Celsius of IMU in main box

#### Output values:
* `rudder_angle` Angle of the rudder, measured in degrees, with 0 straight back and positive turning the boat right. (Float32)
* `propeller_power` Power provided to the propeller, with 0 being no power and 1 being full power. (Float32)

#### Simulator interface
* `motor_powers` (type MotorPower) contains four Float32 values in Newtons: frontleft, frontright, backleft, backright
* `camera/image_raw` (type Image) comes from the front of the boat and has calibration data and stuff
* `lidar/scan` (type sensor_msgs/LaserScan) comes from 50cm off the water flat to the boat.
* `proximity/range1...range5` (type sensor_msgs/Range) for front-facing ultrasonics at 20 degree spacing
* `gps/fix` (type sensor_msgs/NavSatFix) with some reasonable uncertainty included
* `imu` (type sensor_msgs/Imu) with some reasonable uncertainty

#### Cheaty simulator god-mode
* `simulator/location` (type Pose2D) in meters and degrees from simulator origin
