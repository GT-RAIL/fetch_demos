# Fetch Demos
_Demos for the [Fetch platform](http://docs.fetchrobotics.com/index.html) at the [RAIL Lab](http://www.rail.gatech.edu/) @ Georgia Tech_

## Tasks
- Building a navigation map
- Quickly launching the Fetch navigation stack
- Automatically navigate to the Fetch charging station and begin docking
- Track the head of the Fetch robot to the nearest face
- Follow a user around using a combination of face and leg tracking data from the RGBD camera and laser scan

### Building a navigation map

Prerequisites:
- [gmapping](http://wiki.ros.org/gmapping): `sudo apt install ros-indigo-gmapping`

Running:

`roslaunch fetch_demos build_map.launch`

When done mapping, save the map with:

`rosrun map_server map_saver -f <map_directory/map_name>`

The map saver will create two files in the specified map_directory. The directory must already exist. The two files are map_name.pgm and map_name.yaml. The first is the map in a .pgm image format, and the second is a YAML file that specifies metadata for the image.

The `fetch_navigation` provides its own map building launch file but this uses a SLAM library that is prone to crashing. The map building launch file specified here uses a different SLAM library called `gmapping` which fixes this issue.

### Quickly launching the Fetch navigation stack

Running:

`roslaunch fetch_demos navigation.launch`

Will bring up essential services and begin publishing the map from `maps/`, laser scanner, and others. A prerequisite for most Fetch tasks. _Ensure that the directory of the map image indicated by `map.yaml` is correct or else the map server will exit early without a message._

### Automatically navigate to the Fetch charging station and begin docking

Prerequisites:
- `sudo apt-get install ros-indigo-fetch-auto-dock`
- `roslaunch fetch_auto_dock auto_dock.launch`
- If using a new map file, be sure to set the correct dock position statically within `src/dock.py`
- For more information: [see Fetch documentation](http://docs.fetchrobotics.com/docking.html)

Running:
- `rosrun fetch_demos dock.py`
- Will navigate to dock position, begin auto dock procedure, and then exit

### Track the head of the Fetch robot to the nearest face

Prerequisites:
- `rail_face_detection_launch`. We use the Willow Garage face detector by default.

Running:
- `roslaunch fetch_demos track_face.launch`
- Will run the face detection library and point the Fetch head at the nearest detected face in-frame continuously

### Follow a user around

Prerequisites:
- wg-perception's [people library](https://github.com/wg-perception/people): `sudo apt install ros-indigo-people`
- [Fork of the `leg_tracker` library on the `fetch` branch](https://github.com/petschekr/leg_tracker): download to robot's catkin workspace and build with `catkin_make`
    - To just track legs without following people around, run `roslaunch leg_tracker joint_leg_tracker.launch`
    - Documentation of `leg_tracker`: https://github.com/petschekr/leg_tracker
    - This fork configures the leg tracking software for the Fetch's laser scanner and supports OpenCV 2 which is required by other packes on the Fetch platform. The upstream code requires OpenCV 3 which is not currently supported by ROS Indigo on Fetch.
- Navigation stack must be running (map not required): `roslaunch fetch_demos navigation.launch`

Running:
- `roslaunch fetch_demos follow.launch`

The following demo tracks faces using the RGBD camera and then matches the nearest detected face to a pair of legs which it then tracks and moves the robot to follow. The wider field of view of the laser scanner allows for reliable person tracking even with multiple people in the same area or for quick movement. The leg tracking library will also predict leg movements if they go behind an obstacle temporarily. This following code implements filtered collision detection that can differentiate between obstacles in the robot's path and legs which should be tracked.
