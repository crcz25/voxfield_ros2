cmake_minimum_required(VERSION 3.16)
project(voxblox)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(ENABLE_ASAN "Build voxblox with AddressSanitizer/LeakSanitizer" OFF)
if(ENABLE_ASAN AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-fsanitize=address -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)

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

set(PROTO_DEFNS
  proto/voxblox/Block.proto
  proto/voxblox/Layer.proto
)

set(PROTO_SRCS)
set(PROTO_HDRS)
foreach(PROTO_FILE ${PROTO_DEFNS})
  get_filename_component(PROTO_ABS "${PROTO_FILE}" ABSOLUTE)
  get_filename_component(PROTO_NAME_WE "${PROTO_FILE}" NAME_WE)
  set(PROTO_SRC "${CMAKE_CURRENT_BINARY_DIR}/voxblox/${PROTO_NAME_WE}.pb.cc")
  set(PROTO_HDR "${CMAKE_CURRENT_BINARY_DIR}/voxblox/${PROTO_NAME_WE}.pb.h")
  add_custom_command(
    OUTPUT "${PROTO_SRC}" "${PROTO_HDR}"
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
         --proto_path "${CMAKE_CURRENT_SOURCE_DIR}/proto"
         "${PROTO_ABS}"
    DEPENDS "${PROTO_ABS}"
    COMMENT "Generating C++ protobuf files for ${PROTO_FILE}"
    VERBATIM
  )
  list(APPEND PROTO_SRCS "${PROTO_SRC}")
  list(APPEND PROTO_HDRS "${PROTO_HDR}")
endforeach()

add_library(${PROJECT_NAME}_proto ${PROTO_SRCS} ${PROTO_HDRS})
target_include_directories(${PROJECT_NAME}_proto
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}_proto PUBLIC protobuf::libprotobuf)

set(${PROJECT_NAME}_SRCS
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/integrator/np_tsdf_integrator.cc
  src/integrator/esdf_voxfield_integrator.cc
  src/integrator/esdf_occ_fiesta_integrator.cc
  src/integrator/esdf_occ_edt_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
  src/utils/neighbor_tools_ex.cc
)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_proto
    Eigen3::Eigen
    ${GLOG_TARGET}
    ${GFLAGS_TARGET}
    protobuf::libprotobuf
)

add_executable(tsdf_to_esdf test/tsdf_to_esdf.cc)
target_link_libraries(tsdf_to_esdf PRIVATE ${PROJECT_NAME})

add_executable(test_load_esdf test/test_load_esdf.cc)
target_link_libraries(test_load_esdf PRIVATE ${PROJECT_NAME})

install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_proto
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS tsdf_to_esdf test_load_esdf
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/voxblox"
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.pb.h"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 gflags glog Protobuf)
ament_package()
