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

    globalQueryPointsDis_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dis_out", 10);
    globalQueryPointsGrd_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("grd_out", 10);

    //std::string file_path = "/home/lan/Downloads/sydney_harbour_shrink_z.ply";  // Change this to your PLY file path
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

    // openvdb grid initialisation for raw point cloud
    raw_grid_ = openvdb::FloatGrid::create(-100); // default value for each voxel
    raw_grid_->setName("R(x): point cloud grid");
    raw_grid_->setTransform(
      openvdb::math::Transform::createLinearTransform(100));
    raw_grid_->setGridClass(openvdb::GRID_LEVEL_SET);
    
    // not used
    // openvdb grid initialisation for distance field
    dis_grid_ = openvdb::FloatGrid::create(-100); // default value for each voxel
    dis_grid_->setName("D(x): distance grid");
    dis_grid_->setTransform(
      openvdb::math::Transform::createLinearTransform(100));
    dis_grid_->setGridClass(openvdb::GRID_LEVEL_SET);

    // not used
    // openvdb grid initialisation for gradient
    grd_grid_ = openvdb::Vec3fGrid::create(openvdb::Vec3f(0.0f, 0.0f, 0.0f));
    grd_grid_->setName("G(x): gradient grid");
    grd_grid_->setTransform(
      openvdb::math::Transform::createLinearTransform(100));
    grd_grid_->setGridClass(openvdb::GRID_UNKNOWN);

    // Get the transform and the "unsafe" version of the grid accessors
    const openvdb::math::Transform &xformr = raw_grid_->transform();
    auto raw_grid_acc = raw_grid_->getUnsafeAccessor();

    // Get the transform and the "unsafe" version of the grid accessors
    const openvdb::math::Transform &xformd = dis_grid_->transform();
    auto dis_grid_acc = dis_grid_->getUnsafeAccessor();

    // Get the transform and the "unsafe" version of the grid accessors
    const openvdb::math::Transform &xformg = grd_grid_->transform();
    auto grd_grid_acc = grd_grid_->getUnsafeAccessor();

    // iterate raw points to put them in openvdb data structure
    for (size_t i = 0; i < cloud.points.size(); ++i)
    {
        pcl::PointXYZ& point = cloud.points[i];
        // access point coordinates
        openvdb::Vec3d voxeltemp(point.x,point.y,point.z); 
        voxeltemp = xformr.worldToIndex(voxeltemp);
        openvdb::math::Coord localPoint(voxeltemp.x(),voxeltemp.y(),voxeltemp.z()); 
        raw_grid_acc.setValue(localPoint, 1.0);
    }
    
    int numberLeaf = 0;
    std::vector<Eigen::Vector3d> leafVoxels;

    // iterate openvdb data structure to get points out
    for (openvdb::FloatGrid::TreeType::LeafIter iterL = raw_grid_->tree().beginLeaf(); iterL; ++iterL) {
      numberLeaf ++;
      auto leaf = iterL.getLeaf();
      
      for (auto iterLV = leaf->beginValueOn(); iterLV; ++iterLV){
        auto iterValueOnWorld = xformr.indexToWorld(iterLV.getCoord());
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(iterValueOnWorld.x());
        pt.y = static_cast<float>(iterValueOnWorld.y());
        pt.z = static_cast<float>(iterValueOnWorld.z());
        cloud_out.push_back(pt);
        Eigen::Vector3d tmp123(iterValueOnWorld.x(),iterValueOnWorld.y(),iterValueOnWorld.z());
        leafVoxels.push_back(tmp123);
        //std::cout << "leafVoxels: " << iterValueOnWorld.x() << ";" << iterValueOnWorld.y() << ";" << iterValueOnWorld.z() << std::endl;
      }
    }
    std::cout << "cloud_in_vdb: " << cloud_out.size() << std::endl;
    //std::cout << "numberLeaf: " << numberLeaf << std::endl;

    sensor_msgs::msg::PointCloud2 ros_cloud_out;
    pcl::toROSMsg(cloud_out, ros_cloud_out);
    ros_cloud_out.header.frame_id = "map";
    publisherOut_->publish(ros_cloud_out);
    


    // generate a queyring cube in the space
    int interval = 1000; // make this interval as 100 or 200 to have the full distance field and gradients
    std::vector<Eigen::Vector3d> voxelsToUpdate;
    for(double xIdx = -30000; xIdx < 30000; xIdx = xIdx + interval){
      for(double zIdx = -30000; zIdx < 30000; zIdx = zIdx + interval){
        for(double yIdx = -2000; yIdx < 9000; yIdx = yIdx + interval){
          Eigen::Vector3d tmp(xIdx,yIdx,zIdx);
          voxelsToUpdate.push_back(tmp);
        }
      }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS(new pcl::PointCloud<pcl::PointXYZ>);
    size_t voxelsToUpdate_size = voxelsToUpdate.size();
    // Scale factor to avoid floating-point precision issues
    double scale_factor = 0.001;  // Convert meters to kilometers (or adjust as needed)

    for (size_t testIdx = 0; testIdx < voxelsToUpdate_size; testIdx = testIdx+1) {
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(voxelsToUpdate[testIdx].x()* scale_factor);
      pt.y = static_cast<float>(voxelsToUpdate[testIdx].y()* scale_factor);
      pt.z = static_cast<float>(voxelsToUpdate[testIdx].z()* scale_factor);
      cloudS->push_back(pt);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT(new pcl::PointCloud<pcl::PointXYZ>);
    size_t leafVoxels_size = leafVoxels.size();
    for (size_t targetIdx = 0; targetIdx < leafVoxels_size; targetIdx++) {
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(leafVoxels[targetIdx].x()* scale_factor);
      pt.y = static_cast<float>(leafVoxels[targetIdx].y()* scale_factor);
      pt.z = static_cast<float>(leafVoxels[targetIdx].z()* scale_factor);
      cloudT->push_back(pt);
    }

    //std::vector<int> indicesKD;
    //std::vector<float> distancesKD;
    int knne = 1; 

    std::vector<int> indicesKD(voxelsToUpdate.size(), -1);
    std::vector<float> distancesKD(voxelsToUpdate.size(), std::numeric_limits<float>::max());

    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloudT);

    for (size_t idxSource = 0; idxSource < voxelsToUpdate_size; idxSource++)
    {
      std::vector<int> pointIdxNKNSearch;
      std::vector<float> pointNKNSquaredDistance;
      if (kdtree->nearestKSearch(cloudS->points[idxSource], knne, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        indicesKD[idxSource] = pointIdxNKNSearch[0];
        distancesKD[idxSource] = std::sqrt(pointNKNSquaredDistance[0])/ scale_factor;
      }
    }
    std::cout << "finish query: " << distancesKD.size() << std::endl;

    // visual the queying distance
    pcl::PointCloud<pcl::PointXYZI> map_cloud;
    for (size_t i = 0; i < voxelsToUpdate.size(); i++) {
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(voxelsToUpdate[i].x());
      pt.y = static_cast<float>(voxelsToUpdate[i].y());
      pt.z = static_cast<float>(voxelsToUpdate[i].z());
      pt.intensity = static_cast<float>(abs(distancesKD[i]));
      //std::cout << "pt.intensity: " << pt.intensity << ";" << distancesKD[i] << std::endl;
      if(std::isnan(abs(distancesKD[i]))|std::isinf(abs(distancesKD[i]))){ // skip points with bad distance
        continue;
      }else{
        map_cloud.push_back(pt);
      }
    }
    auto map_ptr = map_cloud.makeShared();
    sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    globalQueryPointsDis_pub_->publish(*map_msg_ptr);

    // put the distance values in the grid
    for (size_t i = 0; i < voxelsToUpdate.size(); i++) {
      openvdb::math::Vec3d centerPoint(voxelsToUpdate[i].x(),voxelsToUpdate[i].y(),voxelsToUpdate[i].z());
      openvdb::math::Vec3d centerPointI = xformd.worldToIndex(centerPoint);
      openvdb::math::Coord voxel(centerPointI.x(),centerPointI.y(),centerPointI.z());
      dis_grid_acc.setValue(voxel,distancesKD[i]);
    }

    // Compute the gradient at each voxel using central difference
    // put the gradient in the grid
    int indexall = 1;
    visualization_msgs::msg::MarkerArray mArray;
    for (int xIdx = -30000; xIdx < 30000; xIdx += interval) {
      for (int zIdx = -30000; zIdx < 30000; zIdx += interval) {
          for (int yIdx = -2000; yIdx < 9000; yIdx += interval) {
              
              // Use central difference for gradient approximation
              openvdb::math::Vec3d centerPoint(xIdx,yIdx,zIdx);
              openvdb::math::Vec3d centerPointI = xformd.worldToIndex(centerPoint);
              openvdb::math::Coord voxel0(centerPointI.x(),centerPointI.y(),centerPointI.z());

              openvdb::math::Vec3d centerPoint1(xIdx+interval,yIdx,zIdx);
              openvdb::math::Vec3d centerPoint2(xIdx-interval,yIdx,zIdx);
              openvdb::math::Vec3d centerPoint3(xIdx,yIdx+interval,zIdx);
              openvdb::math::Vec3d centerPoint4(xIdx,yIdx-interval,zIdx);
              openvdb::math::Vec3d centerPoint5(xIdx,yIdx,zIdx+interval);
              openvdb::math::Vec3d centerPoint6(xIdx,yIdx,zIdx-interval);

              openvdb::math::Vec3d centerPointI1 = xformd.worldToIndex(centerPoint1);
              openvdb::math::Coord voxel1(centerPointI1.x(),centerPointI1.y(),centerPointI1.z());

              openvdb::math::Vec3d centerPointI2 = xformd.worldToIndex(centerPoint2);
              openvdb::math::Coord voxel2(centerPointI2.x(),centerPointI2.y(),centerPointI2.z()); 

              openvdb::math::Vec3d centerPointI3 = xformd.worldToIndex(centerPoint3);
              openvdb::math::Coord voxel3(centerPointI3.x(),centerPointI3.y(),centerPointI3.z()); 

              openvdb::math::Vec3d centerPointI4 = xformd.worldToIndex(centerPoint4);
              openvdb::math::Coord voxel4(centerPointI4.x(),centerPointI4.y(),centerPointI4.z()); 

              openvdb::math::Vec3d centerPointI5 = xformd.worldToIndex(centerPoint5);
              openvdb::math::Coord voxel5(centerPointI5.x(),centerPointI5.y(),centerPointI5.z()); 

              openvdb::math::Vec3d centerPointI6 = xformd.worldToIndex(centerPoint6);
              openvdb::math::Coord voxel6(centerPointI6.x(),centerPointI6.y(),centerPointI6.z()); 

              float dx = (dis_grid_acc.getValue(voxel1) - dis_grid_acc.getValue(voxel2)) / 2*interval;
              float dy = (dis_grid_acc.getValue(voxel3) - dis_grid_acc.getValue(voxel4)) / 2*interval;
              float dz = (dis_grid_acc.getValue(voxel5) - dis_grid_acc.getValue(voxel6)) / 2*interval;

              // Store the gradient in Eigen format
              double gradLen = sqrt(dx*dx + dy*dy + dz*dz); 
    
              if(gradLen != 0){
                  dx/=gradLen;
                  dy/=gradLen;
                  dz/=gradLen;
              } else {
                  dx=0;
                  dy=0;
                  dz=0;
              }
              openvdb::Vec3f gradient(dx, dy, dz);
              grd_grid_acc.setValue(voxel0, gradient);
              
              // Optionally, print the gradient for debugging
              // std::cout << "Distance and Gradient at (" << xIdx << ", " << yIdx << ", " << zIdx << ") : (" 
              //           << gradient.x() << ", " << gradient.y() << ", " << gradient.z() << ") (" 
              //           << dis_grid_acc.getValue(voxel0) << ")" << std::endl;

              // if you dont care about magnitude, normalize the gradient vector (looks better)
              geometry_msgs::msg::Point start;
              start.x = xIdx; 
              start.y = yIdx; 
              start.z = zIdx;
              float vecLen1 = 1000; // scales the vector to 1000 ASSUMING it was normalized before
              geometry_msgs::msg::Point end;
              end.x = start.x + dx*vecLen1; 
              end.y = start.y + dy*vecLen1; 
              end.z = start.z + dz*vecLen1;
              float colorGra[] = {0,1,1,1}; // RGBA. Calculate a colormap based on distance to color it according to distance field 
              mArray.markers.push_back(create_arrow(200, start, end, indexall, colorGra));

              indexall ++;
          }
      }
    } 
    globalQueryPointsGrd_pub_->publish(mArray);

    // Create a VDB file
    openvdb::io::File file1("raw_grid.vdb");

    // Create a grid vector and add our grid
    openvdb::GridPtrVec rawgrids;
    rawgrids.push_back(raw_grid_);

    // Write the grids to the file
    file1.write(rawgrids);
    file1.close();
    
    // Create a VDB file
    openvdb::io::File file2("dis_grid.vdb");

    // Create a grid vector and add our grid
    openvdb::GridPtrVec disgrids;
    disgrids.push_back(dis_grid_);

    // Write the grids to the file
    file2.write(disgrids);
    file2.close();

    // Create a VDB file
    openvdb::io::File file3("grd_grid.vdb");

    // Create a grid vector and add our grid
    openvdb::GridPtrVec grdgrids;
    grdgrids.push_back(grd_grid_);

    // Write the grids to the file
    file3.write(grdgrids);
    file3.close();

    std::cout << "Saved all VDB files: raw_grid.vdb, dis_grid.vdb, grd_grid.vdb" << std::endl;
  }

private:
  void topic_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  
  // this function is used if we have service for outside query
  void visualQueriedDistances(const std::vector<float> queryPoints, const std::vector<double> pRes){
    pcl::PointCloud<pcl::PointXYZI> queryPointsPCL;
    int N_pts = queryPoints.size()/3;
    for (size_t ii = 0; ii < N_pts; ii++) {
      int k3 = ii * 3;
      int k8 = ii * 8;
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(queryPoints[k3]);
      pt.y = static_cast<float>(queryPoints[k3+1]);
      pt.z = static_cast<float>(queryPoints[k3+2]);
      //pt.z = static_cast<float>(pRes[k8]+0.9);
      pt.intensity = static_cast<float>(pRes[k8]);
      if(pRes[k8]<=0){ // skip points with bad distance
        continue;
      }else{
        queryPointsPCL.push_back(pt);
      }
    }
    auto map_ptr = queryPointsPCL.makeShared();
    sensor_msgs::msg::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = "map";
    globalQueryPointsDis_pub_->publish(*map_msg_ptr);
  }

  // this function is used if we have service for outside query
  void visualQueriedGradients(const std::vector<float> queryPoints, const std::vector<double> pRes){
    visualization_msgs::msg::MarkerArray mArray;
    int N_pts = queryPoints.size()/3;
    for(int idx = 0; idx < N_pts; idx++) {
        int k3 = idx * 3;
        int k8 = idx * 8;
        // if you dont care about magnitude, normalize the gradient vector (looks better)
        geometry_msgs::msg::Point start;
        start.x = queryPoints[k3]; 
        start.y = queryPoints[k3+1]; 
        start.z = queryPoints[k3+2];
        float vecLen1 = 0.4; // scales the vector to 40cm ASSUMING it was normalized before
        geometry_msgs::msg::Point end;
        end.x = start.x + pRes[k8+1]*vecLen1; 
        end.y = start.y + pRes[k8+2]*vecLen1; 
        end.z = start.z + pRes[k8+3]*vecLen1;
        float colorGra[] = {0,1,1,1}; // RGBA. Calculate a colormap based on distance to color it according to distance field 
        mArray.markers.push_back(create_arrow(0.04, start, end, idx, colorGra));
    }
    globalQueryPointsGrd_pub_->publish(mArray);
  }
 
  visualization_msgs::msg::Marker create_arrow(float scale, 
      const geometry_msgs::msg::Point &start, 
      const geometry_msgs::msg::Point &end, 
      int idnum, 
      const float color[4]) {
    visualization_msgs::msg::Marker m;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.header.frame_id = "map"; // Replace with your world frame if necessary
    m.header.stamp = this->get_clock()->now();
    m.id = idnum;
    m.type = visualization_msgs::msg::Marker::ARROW;

    // Default orientation (no rotation)
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;

    m.scale.x = scale*1.0;       // Thickness of the arrow shaft
    m.scale.y = scale*2.0;   // Thickness of the base
    m.scale.z = scale*0.5;         // Arrowhead scale

    // Set color
    m.color.r = color[0];
    m.color.g = color[1];
    m.color.b = color[2];
    m.color.a = color[3];

    // Add start and end points
    m.points.push_back(start);
    m.points.push_back(end);

    return m;
  }

  // publishers ad subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherIn_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherOut_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalQueryPointsDis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr globalQueryPointsGrd_pub_;

  // OpenVDB Grids to store the point cloud
  openvdb::FloatGrid::Ptr raw_grid_;
  openvdb::FloatGrid::Ptr dis_grid_;
  openvdb::Vec3fGrid::Ptr grd_grid_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
