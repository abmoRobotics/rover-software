# Space Interface.
This is the documentation for the custom msg and srv files. 
## msg

### Marker.msg
**Description**: Msg for an induvidual aruco marker with ID and pose.
int64 id
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

### MarkerList.msg
**Description**: Msg for a list of aruco markers.
space_interface/Marker[] markers

## srv
