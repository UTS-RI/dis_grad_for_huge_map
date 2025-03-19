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

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

#include "OnGPDF.h"

using std::placeholders::_1;
  
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    publisherIn_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_in", 10);
    publisherOut_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_out", 10);

    std::string file_path = "/home/lan/Downloads/SydneyHarbourSubmapMesh.ply";  // Change this to your PLY file path
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(file_path, cloud) == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to load PLY file: %s", file_path.c_str());
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %d points from PLY file", cloud.size());

    sensor_msgs::msg::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header.frame_id = "map";  // Set the coordinate frame
    publisherIn_->publish(ros_cloud);

    // openvdb grid initialisation
    gsdf_ = openvdb::FloatGrid::create(-100); // default value for each voxel
    gsdf_->setName("D(x): point cloud grid");
    gsdf_->setTransform(
      openvdb::math::Transform::createLinearTransform(110));
    gsdf_->setGridClass(openvdb::GRID_LEVEL_SET);
   
    // Get the transform and the "unsafe" version of the grid accessors
    const openvdb::math::Transform &xform = gsdf_->transform();
    auto gsdf_acc = gsdf_->getUnsafeAccessor();

    // iterate raw points to put them in openvdb data structure
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        pcl::PointXYZ& point = cloud.points[i];
        
        // access point coordinates
        openvdb::Vec3d voxeltemp(point.x,point.y,point.z); 
        voxeltemp = xform.worldToIndex(voxeltemp);
        openvdb::math::Coord localPoint(voxeltemp.x(),voxeltemp.y(),voxeltemp.z()); 
        gsdf_acc.setValue(localPoint, 1.0);
    }
    
    std::vector<std::shared_ptr<OnGPDF>> localGPs;
    std::vector<Eigen::Vector3d> localGPsCenters;
    int distance_methods = 0;
    double map_lambda_scales = 200;
    double map_noises = 0.5;
    int numberLeaf = 0;

    // iterate openvdb data structure to get points out
    for (openvdb::FloatGrid::TreeType::LeafIter iterL = gsdf_->tree().beginLeaf(); iterL; ++iterL) {
      numberLeaf ++;
      auto leaf = iterL.getLeaf();
      
      const openvdb::CoordBBox bboxTest = iterL->getNodeBoundingBox();
      const auto coordBBoxIntO = xform.indexToWorld(bboxTest);
      
      std::vector<Eigen::Vector3d> leafVoxels;
      for (auto iterLV = leaf->beginValueOn(); iterLV; ++iterLV){
        auto iterValueOnWorld = xform.indexToWorld(iterLV.getCoord());
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(iterValueOnWorld.x());
        pt.y = static_cast<float>(iterValueOnWorld.y());
        pt.z = static_cast<float>(iterValueOnWorld.z());
        cloud_out.push_back(pt);
        Eigen::Vector3d tmp123(iterValueOnWorld.x(),iterValueOnWorld.y(),iterValueOnWorld.z());
        leafVoxels.push_back(tmp123);

        // std::shared_ptr<OnGPDF> gp(new OnGPDF(leafVoxels, distance_methods, map_lambda_scales, map_noises));
        // openvdb::Vec3R leafCenter = coordBBoxIntO.getCenter();
        // Eigen::Vector3d gpCenter(leafCenter.x(),leafCenter.y(),leafCenter.z());
        // std::vector<Eigen::Vector3d>::iterator iter=std::find(localGPsCenters.begin(),localGPsCenters.end(),gpCenter);
        // if(iter == localGPsCenters.end())
        // {
        //   localGPsCenters.push_back(gpCenter);
        //   localGPs.push_back(gp);
        // } else {
        //   auto idxToUpdate = std::distance(localGPsCenters.begin(),iter);
        //   localGPs[idxToUpdate] = gp;
        // }
      }
    }
    std::cout << "cloud_in_vdb: " << cloud_out.size() << std::endl;
    std::cout << "numberLeaf: " << numberLeaf << std::endl;

    sensor_msgs::msg::PointCloud2 ros_cloud_out;
    pcl::toROSMsg(cloud_out, ros_cloud_out);
    ros_cloud_out.header.frame_id = "map";  // Set the coordinate frame
    publisherOut_->publish(ros_cloud_out);
    
    // RCLCPP_INFO(this->get_logger(), "local_GPs_number: %d ", localGPsCenters.size());
    // for (size_t gpIdx = 0; gpIdx < localGPs.size(); gpIdx ++) {
    //   std::vector<Eigen::Vector3d> leafVoxelss;
  
    //   auto loopGP = localGPs[gpIdx];
    //   leafVoxelss = loopGP->getPoints();
      
    //   std::shared_ptr<OnGPDF> gp1(new OnGPDF(leafVoxelss,distance_methods,map_lambda_scales,map_noises));
    //   gp1->train(leafVoxelss);
    //   localGPs[gpIdx] = gp1;
    // }
    // RCLCPP_INFO(this->get_logger(), "local_GPs_number: %d ", localGPsCenters.size());
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  // publishers ad subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherIn_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherOut_;

  // OpenVDB Grids to store the point cloud
  openvdb::FloatGrid::Ptr gsdf_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
