# tuw_calibration

Package including ROS nodes to calibrate the pose of a camera mounted on a mobile robot using a checkerboard attached to a wall near a corner and a laser scanner with a known transform to the robot base.

The following image shows the general setup for the calibration. A checkerboard has to be placed at a wall near a corner with the robot upfront facing the wall. Then the height of the laser scanner (z_W,Co), the checkerboard height (z_Co,C) and the distance of the checkerboard to the corner (y_Co,C) have to be measured and provided to the tuw_calib_base2cam node.

![calibration_setup tag](https://github.com/tuw-robotics/tuw_calibration/blob/master/pics/calibration.png)

The following nodes are provided:

### **tuw_linedetection:**
Wrapper node for the line detection algorithm provided by [tuw_geometry](https://github.com/tuw-robotics/tuw_geometry), publishing detected line segments as line segment messages (see [tuw_msgs](https://github.com/tuw-robotics/tuw_msgs) and [tuw_rviz](https://github.com/tuw-robotics/tuw_rviz))

#### **Subscribed Topics:**
* scan(sensor_msgs/LaserScan)

#### **Published Topics:**
* line_segments(tuw_geometry_msgs/LineSegments)

#### **Parameters:**
* laser_frame(str, default="/p3dx/laser_front")

### **tuw_checkerboard:**
Uses checkerboard detection from OpenCV and solvePnP to create a ROS tf from the checkerboard to the camera. Also uses the height of the checkerboard w.r.t. the ground and its distance to the corner to publish the tf from the corner to the checkerboard.
This node is now included in [tuw_marker_detection](https://github.com/tuw-robotics/tuw_marker_detection/tree/kinetic-devel).

### **tuw_laser2corner:**
Gets the line segments from the line detection node and a point near the corner. Then uses the two line segments closest to the corner point to determine the tf from the laser to the corner.

#### **Subscribed Topics:**
* line_segments(tuw_geometry_msgs/LineSegments)

#### **Provided tf Transforms:**
* laser --> corner

#### **Parameters:**
* laser_frame(str, default=/p3dx/laser_front)
* corner_point_tolerance(double, 0.005) - tolerance in which line segment end points have to match to form a corner point
* corner_point_x(double, 1) - corner point heuristic x coordinate
* corner_point_y(double, -1) - corner point heuristic y coordinate

### **tuw_calib_base2cam:**
Puts the tf's laser --> corner, corner --> checkerboard, checkerboard --> camera, together and outputs tf parameters for a static camera to robot base transform to stdout.

#### **Required tf Transforms:**
* camera_link --> checkerboard
* base_link --> corner

#### **Parameters:**
* camera_link(str, default="/camera_link")
* base_link(str, default="/base_link")
* checkerboard_frame(str, default="/checkerboard")
* corner_frame(str, default="/corner")
* laser_height(double, default=0.3) - height of the laser scanner from the ground
* checker_height(double, default=1.295) - height of the checkerboard from the ground
* checker_y(double, default=0.475) - distance between checkerboard origin and corner

## Usage
The launch file for tuw_calib_base_cam also launches the other required nodes. One can customize topics and parameters using the launch file arguments.

`roslaunch tuw_calib_base2cam tuw_calib_base2cam.launch`
