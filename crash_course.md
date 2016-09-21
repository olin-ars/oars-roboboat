# How to make the boat go

## ssh into boat
- cd to `ors-git-repo/shell_scripts`
- connect to bat with `./connect_to_fitpc.sh`

## start code
- for simulator: `roslaunch oars_pkg simulator_auto.launch`

## launch webpage thing
- cd to `ors-git-repo/fit_pc_pkg/scritps/html-gui`
  - alternately, from anywhere `roscd fit_pc_pg/scripts/html-gui`
- use a web browser to open `index.html`
  - (in my case `chromium-web-browser index.html`)
- there is a prompt, the default number is correct for connecting to the boat
  - for running simulator, change that to `localhost`

## change mode
- `rostopic pub /operating_mode std_msgs/Int16 "data: #"
- or go to `ors-git-repo/shell_scripts` and run `sh op_mode.sh #`
  - key:
    - 0: testing
    - 1: RC
    - 2: auto
    - 3: semi-auto (autonomous sails, rc rudder)
    - 4: station keep

## set waypoint

### waypoint using lat-lon
- use this command:
``` 
rostopic pub /raw_waypoints geometry_msgs/Pose2D "x: lat 
y: lon 
theta: 1"
```
- ps. use tab-complete
- alternately, go to `ors-git-repo/shell_scripts` and run `make_gps_wp.sh lat lon`

### waypoint using local coordinates
- use this command:
```
rostopic pub /local_waypoints geometry_msgs/Pose2D "x: east
y: north
theta: 1"
```
- ps. use tab-complete
- alternately, go to `ors-git-repo/shell_scripts` and run `make_local_wp.sh east north`

### other waypoint things
- boolean publishers follow the format `rostopic pub <topic> std_msgs/Bool "data: <true/flase>"`
- publish `true` to `/clear_waypoints` to clear waypoints
- publish `true` to `/rm_waypoint` to remove the most recently added waypoint
- publish `true` to `/skip_waypoint` to skip the next waypoint set for the boat
