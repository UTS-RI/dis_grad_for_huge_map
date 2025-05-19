// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "edf_srv/srv/query_edf.hpp"

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>

// OpenVDB
#include <openvdb/Types.h>
#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/math/Transform.h>
#include <openvdb/tree/LeafManager.h>
#include <openvdb/tree/LeafNode.h>
#include <openvdb/tree/Tree.h>
#include <openvdb/math/Coord.h>
#include <openvdb/tree/LeafNodeBool.h>
#include <openvdb/tools/Activate.h>
#include <openvdb/tools/LevelSetUtil.h>
#include <openvdb/tools/Interpolation.h>

#include "OnGPDF.h"

namespace edf_mapping {
class EDFMapper {
    public:

    EDFMapper(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

}
}