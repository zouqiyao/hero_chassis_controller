cmake_minimum_required(VERSION 3.10)
project(hero_chassis_controller)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED OFF)

## Treat warnings as errors, like the rm_template this package was created from
add_definitions(-Wall -Werror)

## Find catkin macros and libraries.
## roslint is build-only, everything else must match package.xml.
find_package(catkin REQUIRED
    COMPONENTS
    roscpp
    roslint
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    )

## Find system libraries (Eigen is used for kinematics / odometry math)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## INCLUDE_DIRS is exported to dependent packages. Eigen3 only provides the
## non-standard EIGEN3_INCLUDE_DIR variable, so it has to be added manually.
catkin_package(
    INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    controller_interface
    hardware_interface
    pluginlib
    control_toolbox
    geometry_msgs
    nav_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)

###########
## Build ##
###########

include_directories(
    include

    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

## A ros_control controller is a plugin, so it must be built as a shared
## library (libhero_chassis_controller.so), NOT as an executable node.
add_library(${PROJECT_NAME}
    src/hero_chassis_controller.cpp
    )

add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
    )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

## pluginlib needs the plugin description XML inside the package share dir
install(
    FILES hero_chassis_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
    DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

##########################
## Static code analysis ##
##########################

roslint_cpp()
