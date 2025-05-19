# Euclidean distance field and gradients for map

## Installation

To install, first please install OpenVDB following these instructions: https://github.com/AcademySoftwareFoundation/openvdb

After OpenVDB is installed, clone the repo into your ROS2 workspace and re-build your workspace
```
colcon build <workspace_name> && source install/setup.bash
```
## Usage

Start the ROS2 node with:
```
ros2 run edf_map edf_service
```
Query the distance and direction to the closest obstacle with:
```
ros2 service call /edf_srv edf_srv/srv/QueryEdf "{points: [x1,y1,z1,x2,y2,z2,...,xN,yN,zN]}"
```
where `x1,y1,z2, etc...` are the query locations you are interested in. The service will return a list of distances and gradients corresponding to your input.

### Topics

Additionally the `/dis_out` and `/grd_out` topics are published to help visualise the distance field at your query points.

`/dis_out` is a PointCloud at your query points with intensity values set to the corresponding distances to closest obstacle.

`/grd_out` is a set of vectors aligned with the `/dis_out` PointCloud pointing in the direction of the closest obstacle.
