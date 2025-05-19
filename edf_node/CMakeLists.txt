cmake_minimum_required(VERSION 3.8)
project(sydney_edf_map)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(edf_srv REQUIRED)

#set(OpenVDB_DIR "/usr/local/lib/cmake/OpenVDB")
list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/cmake/OpenVDB")
find_package(OpenVDB REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

# add_executable(talker src/publisher_member_function.cpp)
# ament_target_dependencies(talker rclcpp std_msgs edf_srv)

# install(TARGETS
#   talker
#   DESTINATION lib/${PROJECT_NAME})

add_executable(edf_service src/subscriber_member_function.cpp)
ament_target_dependencies(edf_service rclcpp std_msgs rclcpp sensor_msgs pcl_conversions PCL visualization_msgs edf_srv)

install(TARGETS
  talker
  edf_service
  DESTINATION lib/${PROJECT_NAME})

# add_library(ongpdf_lib src/OnGPDF.cpp)
# target_link_libraries(listener ${OpenVDB_LIBRARIES} ${catkin_LIBRARIES} ongpdf_lib)

ament_package()
