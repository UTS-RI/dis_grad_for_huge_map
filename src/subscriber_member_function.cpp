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
#include "edf_srv/srv/query_edf_slice.hpp"

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

// #include "OnGPDF.h"

using std::placeholders::_1;
using std::placeholders::_2;
  
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    publisherIn_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_in", 10);
    publisherOut_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud_out", 10);

    globalQueryPointsDis_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dis_out", 10);
    globalQueryPointsGrd_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("grd_out", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MinimalSubscriber::timer_callback, this));

    std::string file_path = "./src/dis_grad_for_huge_map/dupesyd.ply";  // Change this to your PLY file path

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud_out_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    // pcl::PointCloud<pcl::PointXYZ> cloud_out;
    
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
      openvdb::math::Transform::createLinearTransform(100)); // voxel size 100 units
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
    const openvdb::math::Transform &xformr = raw_grid_->transform(); //xformr is a reference to the transform
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
        openvdb::Vec3d voxeltemp(point.x,-point.z,point.y);
        // stores them in voxel indexed according to the raw grid transformation ?
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
        // reverses the transformation back to world frame
        auto iterValueOnWorld = xformr.indexToWorld(iterLV.getCoord());
        pcl::PointXYZ pt;
        pt.x = static_cast<float>(iterValueOnWorld.x());
        pt.y = static_cast<float>(iterValueOnWorld.y());
        pt.z = static_cast<float>(iterValueOnWorld.z());
        cloud_out_->push_back(pt);
        Eigen::Vector3d tmp123(iterValueOnWorld.x(),iterValueOnWorld.y(),iterValueOnWorld.z());
        // stores world frame points also in this vector of Vector3ds
        leafVoxels.push_back(tmp123);
      }
    }
    std::cout << "cloud_in_vdb: " << cloud_out_->size() << std::endl;
    
    // publish un-transformed point cloud
    sensor_msgs::msg::PointCloud2 ros_cloud_out;
    pcl::toROSMsg(*cloud_out_, ros_cloud_out);
    ros_cloud_out.header.frame_id = "map";
    publisherOut_->publish(ros_cloud_out);
    
    double scale_factor = 1;  // unit/scale conversion (if needed)

    cloudT_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    size_t leafVoxels_size = leafVoxels.size();
    for (size_t targetIdx = 0; targetIdx < leafVoxels_size; targetIdx++) {
      pcl::PointXYZ pt;
      pt.x = static_cast<float>(leafVoxels[targetIdx].x()* scale_factor);
      pt.y = static_cast<float>(leafVoxels[targetIdx].y()* scale_factor);
      pt.z = static_cast<float>(leafVoxels[targetIdx].z()* scale_factor);
      cloudT_->push_back(pt);
    }

    int knne = 1; 
    // make the kd tree from gt points
    kdtree_ = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    kdtree_->setInputCloud(cloudT_);
    query_edf_srv_ = this->create_service<edf_srv::srv::QueryEdf>("edf_srv", 
                                                                  std::bind(&MinimalSubscriber::queryEDF_callback, this, _1,_2));
    slice_edf_srv_ = this->create_service<edf_srv::srv::QueryEdfSlice>("slice_edf_srv",
                                                                  std::bind(&MinimalSubscriber::queryEDF_slice, this, _1,_2));
  }

private:
  std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>> kdtree_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudT_;
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_out_;

  void timer_callback() {
    sensor_msgs::msg::PointCloud2 ros_cloud_out;
    pcl::toROSMsg(*cloudT_, ros_cloud_out);
    ros_cloud_out.header.frame_id = "map";
    publisherOut_->publish(ros_cloud_out);
  }

  void queryEDF_slice(
    const std::shared_ptr<edf_srv::srv::QueryEdfSlice::Request> reqQ,
    std::shared_ptr<edf_srv::srv::QueryEdfSlice::Response> resS)  {
      // generate a queyring cube in the space
      int interval = 100; // make this interval as 100 or 200 to have the full distance field and gradients
      std::vector<Eigen::Vector3d> voxelsToUpdate;
      // change bounds for x and y axes as necessary
      for(double xIdx = -30000; xIdx < 30000; xIdx = xIdx + interval){
        for(double yIdx = -30000; yIdx < 30000; yIdx = yIdx + interval){
            Eigen::Vector3d tmp(xIdx,yIdx,reqQ->z);
            // vector of query points in world coordinates
            voxelsToUpdate.push_back(tmp);
          }
        }
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudS(new pcl::PointCloud<pcl::PointXYZ>);
      size_t voxelsToUpdate_size = voxelsToUpdate.size();
      resS->distances.resize(voxelsToUpdate_size,0);
    // // Scale factor to avoid floating-point precision issues
      double scale_factor = 100;  // Convert meters to kilometers (or adjust as needed)
      for (size_t testIdx = 0; testIdx < voxelsToUpdate_size; testIdx = testIdx+1) {
        pcl::PointXYZ pt;
        // query points in world coords but in kilometres rather than metres
        pt.x = static_cast<float>(voxelsToUpdate[testIdx].x()* scale_factor);
        pt.y = static_cast<float>(voxelsToUpdate[testIdx].y()* scale_factor);
        pt.z = static_cast<float>(voxelsToUpdate[testIdx].z()* scale_factor);
        cloudS->push_back(pt);
      }

      std::vector<int> indicesKD(voxelsToUpdate.size(), -1);
      std::vector<float> distancesKD(voxelsToUpdate.size(), std::numeric_limits<float>::max());

      for (size_t idxSource = 0; idxSource < voxelsToUpdate_size; idxSource++)
      {
        std::vector<int> pointIdxNKNSearch;
        std::vector<float> pointNKNSquaredDistance;
        if (kdtree_->nearestKSearch(cloudS->points[idxSource], knne, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
        {
          indicesKD[idxSource] = pointIdxNKNSearch[0];
          // store distances in metres again
          distancesKD[idxSource] = std::sqrt(pointNKNSquaredDistance[0]) / scale_factor;
        }
      }
      std::cout << "finish query: " << distancesKD.size() << std::endl;

      // visualise the queying distance
      pcl::PointCloud<pcl::PointXYZI> map_cloud;
      for (size_t i = 0; i < voxelsToUpdate.size(); i++) {
        pcl::PointXYZI pt;
        pt.x = static_cast<float>(voxelsToUpdate[i].x());
        pt.y = static_cast<float>(voxelsToUpdate[i].y());
        pt.z = static_cast<float>(voxelsToUpdate[i].z());
        pt.intensity = static_cast<float>(abs(distancesKD[i]));
        if(std::isnan(abs(distancesKD[i]))|std::isinf(abs(distancesKD[i]))){ // skip points with bad distance
          continue;
        }else{
          resS->distances[i] = pt;
        }
      }
      visualQueriedDistances(voxelsToUpdate,resS->distances);
    }

  void queryEDF_callback(
    const std::shared_ptr<edf_srv::srv::QueryEdf::Request> reqQ,
    std::shared_ptr<edf_srv::srv::QueryEdf::Response> resS) {
      
      std::vector<float> queryPoints(reqQ->points.begin(), reqQ->points.end());
      if (queryPoints.empty()) {
        std::cerr << "Query is empty, check again!\n";
      }
      
      auto validNumberTest = queryPoints.size()%3;
      if (validNumberTest != 0) {
        std::cerr << "Query is wrong size, make sure input is (x,y,z) coordinates!\n";
      }

      if (!cloudT_ || cloudT_->empty()) {
        std::cerr << "KD point cloud is empty!\n";
      }

      if (!kdtree_) {
        std::cerr << "KDTree empty or uninitialised!\n";
      }

    int knne = 1; 
    int N_pts = queryPoints.size() / 3;
    
    resS->distances.resize(N_pts,0);
    resS->gradients.resize(N_pts*3,0);

    std::vector<int> indicesKD(N_pts, -1);
    std::vector<float> distancesKD(N_pts, std::numeric_limits<float>::max());

    // iterate through query points
    for (int idxSource = 0; idxSource < N_pts; idxSource++)
    {
      std::vector<int> pointIdxNKNSearch;
      std::vector<float> pointNKNSquaredDistance;
      std::vector<float> q_pt_grads;
      double scale_factor = 100;
      pcl::PointXYZ q_pt(reqQ->points[(idxSource*3)] * scale_factor,
                         reqQ->points[(idxSource*3)+1] * scale_factor,
                         reqQ->points[(idxSource*3)+2] * scale_factor);
      if (kdtree_->nearestKSearch(q_pt, knne, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
      {
        int dx = 100;
        calculateNumericalGrad(q_pt, kdtree_, dx, q_pt_grads);
        resS->distances[idxSource] = std::sqrt(pointNKNSquaredDistance[0]) / scale_factor;
        resS->gradients[idxSource*3] = q_pt_grads[0];
        resS->gradients[(idxSource*3)+1] = q_pt_grads[1];
        resS->gradients[(idxSource*3)+2] = q_pt_grads[2];
      }
      else {
        std::cerr << "Could not find any nearest neighbours\n";
      }
    }
    visualQueriedDistances(queryPoints,resS->distances);
    visualQueriedGradients(queryPoints,resS->gradients);
    }

  void calculateNumericalGrad(const pcl::PointXYZ queryPoint, const std::shared_ptr<pcl::search::KdTree<pcl::PointXYZ>> kdtree, const int dx, std::vector<float> &grad){
    std::vector<pcl::PointXYZ> diff_points;
    diff_points.push_back(pcl::PointXYZ(queryPoint.x + dx, queryPoint.y, queryPoint.z));
    diff_points.push_back(pcl::PointXYZ(queryPoint.x - dx, queryPoint.y, queryPoint.z));
    diff_points.push_back(pcl::PointXYZ(queryPoint.x, queryPoint.y + dx, queryPoint.z));
    diff_points.push_back(pcl::PointXYZ(queryPoint.x, queryPoint.y - dx, queryPoint.z));
    diff_points.push_back(pcl::PointXYZ(queryPoint.x, queryPoint.y, queryPoint.z + dx));
    diff_points.push_back(pcl::PointXYZ(queryPoint.x, queryPoint.y, queryPoint.z - dx));
    int knne = 1;
    double scale_factor = 100;
    grad.resize(3);

    for (int idx = 0; idx < 3; idx++) {
      std::vector<int> plusPointIdxNKNSearch;
      std::vector<float> plusPointNKNSquaredDistance;
      std::vector<int> minusPointIdxNKNSearch;
      std::vector<float> minusPointNKNSquaredDistance;
      kdtree->nearestKSearch(diff_points[idx*2], knne, plusPointIdxNKNSearch, plusPointNKNSquaredDistance);
      kdtree->nearestKSearch(diff_points[(idx*2)+1], knne, minusPointIdxNKNSearch, minusPointNKNSquaredDistance);
      float dis_plus = std::sqrt(plusPointNKNSquaredDistance[0]) / scale_factor;
      float dis_minus = std::sqrt(minusPointNKNSquaredDistance[0]) / scale_factor;
      grad[idx] = static_cast<float>((dis_plus - dis_minus) / (2 * dx));
    }
    double gradLen = sqrt(grad[0]*grad[0] + grad[1]*grad[1] + grad[2]*grad[2]); 
    if(gradLen != 0){
      grad[0]=-grad[0]/gradLen;
      grad[1]=-grad[1]/gradLen; // unreal to ros frame transformation
      grad[2]=-grad[2]/gradLen;
    } else {
      grad[0]=0;
      grad[1]=0;
      grad[2]=0;
    }
  }
  
  // this function is used if we have service for outside query
  void visualQueriedDistances(const std::vector<float> queryPoints, const std::vector<double> pRes){
    pcl::PointCloud<pcl::PointXYZI> queryPointsPCL;
    int N_pts = queryPoints.size()/3;
    double scale_factor = 100;
    for (int ii = 0; ii < N_pts; ii++) {
      int k3 = ii * 3;
      // int k8 = ii * 8;
      pcl::PointXYZI pt;
      pt.x = static_cast<float>(queryPoints[k3])* scale_factor;
      pt.y = static_cast<float>(queryPoints[k3+1])* scale_factor;
      pt.z = static_cast<float>(queryPoints[k3+2])* scale_factor;
      //pt.z = static_cast<float>(pRes[k8]+0.9);
      pt.intensity = static_cast<float>(pRes[ii]);
      if(pRes[ii]<=0){ // skip points with bad distance
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
    double scale_factor = 100;
    for(int idx = 0; idx < N_pts; idx++) {
        int k3 = idx * 3;
        int k8 = idx * 8;
        // if you dont care about magnitude, normalize the gradient vector (looks better)
        geometry_msgs::msg::Point start;
        start.x = queryPoints[k3] * scale_factor; 
        start.y = queryPoints[k3+1] * scale_factor; 
        start.z = queryPoints[k3+2] * scale_factor;
        float vecLen1 = 100; 
        geometry_msgs::msg::Point end;
        end.x = start.x + pRes[k3]*vecLen1; 
        end.y = start.y + pRes[k3+1]*vecLen1; 
        end.z = start.z + pRes[k3+2]*vecLen1;
        float colorGra[] = {0,1,1,1}; // RGBA. Calculate a colormap based on distance to color it according to distance field 
        mArray.markers.push_back(create_arrow(100, start, end, idx, colorGra));
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

  // publishers ad subscribers and server
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherIn_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisherOut_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalQueryPointsDis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr globalQueryPointsGrd_pub_;

  rclcpp::Service<edf_srv::srv::QueryEdf>::SharedPtr query_edf_srv_;
  rclcpp::Service<edf_srv::srv::QueryEdfSlice>::SharedPtr slice_edf_srv_;

  rclcpp::TimerBase::SharedPtr timer_;

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
