cmake_minimum_required(VERSION 2.8.3)
project(pmd_camboard_nano)

find_package(catkin REQUIRED
             COMPONENTS roscpp sensor_msgs image_transport nodelet dynamic_reconfigure tf)

generate_dynamic_reconfigure_options(cfg/PMD.cfg)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs image_transport nodelet dynamic_reconfigure tf
)

# Set PMDSDK Requirements
set(PMDSDK_ROOT_DIR ${PROJECT_SOURCE_DIR}/PMDSDK) # Change this if the path is different

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
find_package(PMDSDK REQUIRED)

#########
# BUILD #
#########
add_definitions(-DPMD_PLUGIN_DIR="${PMDSDK_PLUGIN_DIR}/")
include_directories(include ${catkin_INCLUDE_DIRS} ${PMDSDK_INCLUDE_DIR})

# make sure configure headers are built before any node using them
add_library(pmd_camboard_nano_nodelet src/driver_nodelet.cpp src/pmd_camboard_nano.cpp)
target_link_libraries(pmd_camboard_nano_nodelet ${catkin_LIBRARIES} ${PMDSDK_LIBRARIES})
add_dependencies(pmd_camboard_nano_nodelet ${PROJECT_NAME}_gencfg)

add_executable(pmd_camboard_nano_node src/driver_node)
target_link_libraries(pmd_camboard_nano_node ${catkin_LIBRARIES} ${PMDSDK_LIBRARIES})
add_dependencies(pmd_camboard_nano_node ${PROJECT_NAME}_gencfg)
add_dependencies(pmd_camboard_nano_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
