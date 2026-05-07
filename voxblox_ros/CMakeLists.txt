cmake_minimum_required(VERSION 3.16)
project(voxblox_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(ENABLE_ASAN "Build voxblox_ros with AddressSanitizer/LeakSanitizer" OFF)
if(ENABLE_ASAN AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters kdtree)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)

if(TARGET gflags::gflags)
  set(GFLAGS_TARGET gflags::gflags)
else()
  set(GFLAGS_TARGET gflags)
endif()

if(TARGET glog::glog)
  set(GLOG_TARGET glog::glog)
else()
  set(GLOG_TARGET glog)
endif()

add_library(${PROJECT_NAME}
  src/ros_params.cc
  src/transformer.cc
  src/np_tsdf_server.cc
  src/voxfield_server.cc
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_conversions
  voxblox
  voxblox_msgs
)

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GLOG_TARGET}
  ${GFLAGS_TARGET}
)

add_executable(np_tsdf_server src/np_tsdf_server_node.cc)
target_link_libraries(np_tsdf_server PRIVATE ${PROJECT_NAME})

add_executable(voxfield_server src/voxfield_server_node.cc)
target_link_libraries(voxfield_server PRIVATE ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} np_tsdf_server voxfield_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch cfg DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  pcl_conversions
  voxblox
  voxblox_msgs
)

ament_package()
