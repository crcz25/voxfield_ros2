cmake_minimum_required(VERSION 3.16)
project(voxblox_rviz_plugin)

option(BUILD_VOXBLOX_RVIZ_PLUGIN "Build Voxblox RViz2 plugin" ON)

if(NOT BUILD_VOXBLOX_RVIZ_PLUGIN)
  find_package(ament_cmake REQUIRED)
  ament_package()
  return()
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_AUTOMOC ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(voxblox REQUIRED)
find_package(voxblox_msgs REQUIRED)

add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES
  include/voxblox_rviz_plugin/material_loader.h
  include/voxblox_rviz_plugin/voxblox_mesh_display.h
  include/voxblox_rviz_plugin/voxblox_mesh_visual.h
  include/voxblox_rviz_plugin/voxblox_multi_mesh_display.h
)

set(SRC_FILES
  src/material_loader.cc
  src/voxblox_mesh_display.cc
  src/voxblox_mesh_visual.cc
  src/voxblox_multi_mesh_display.cc
)

add_library(${PROJECT_NAME} SHARED
  ${SRC_FILES}
  ${HEADER_FILES}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    ament_index_cpp
    pluginlib
    rclcpp
    rviz_common
    rviz_rendering
    voxblox
    voxblox_msgs
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Qt5::Widgets
    rviz_ogre_vendor::OgreMain
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
register_rviz_ogre_media_exports(DIRECTORIES content)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY content icons
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  pluginlib
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  voxblox
  voxblox_msgs
)

ament_package()
