# Perception Stack documentation.
## aruco_extractor_node.py
### Overview
Takes the image feed and sends a list of Aruco Markers, their ID, and Pose. If further info is needed then the Marker msg can be updated. 
### functionality
* Subscribes to the topic **"/camera_raw"** of the msg type **sensor_msgs.msg->Image**
* Publishes the custom msg **space_interface.msg->MarkerList** on the topic **/detected_aruco_markers**
### Testing 
To test the speed camera node that publishes an image of Aruco markers can be used. 
Open a terminal. 
```bash
colcon build && source install/setup.bash && ros2 run perception_stack pseodu_camera_delete.py
```
New terminal

```bash
colcon build && source install/setup.bash && ros2 run perception_stack aruco_extractor_node.py
```
New terminal.
```bash
source install/setup.bash
ros2 topic echo /detected_aruco_markers 
```
