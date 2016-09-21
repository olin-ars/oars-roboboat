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


#### Output values:
* `rudder_angle` Angle of the rudder, measured in degrees, with 0 straight back and positive turning the boat right. (Float32)
* `propeller_power` Power provided to the propeller, with 0 being no power and 1 being full power. (Float32)

