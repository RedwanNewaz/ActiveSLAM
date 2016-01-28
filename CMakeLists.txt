cmake_minimum_required(VERSION 2.8.3)
project(active_slam)

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  octomap_msgs
  tf
  image_transport
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV)
find_package(octomap 1.5 REQUIRED)


#add service
add_service_files(
  DIRECTORY srv
  FILES
  pidgain.srv
  plannertalk.srv
  obstacle.srv
  sensor.srv
)

## Generate added messages
generate_messages(DEPENDENCIES std_msgs)


# do QT stuff
ADD_DEFINITIONS( -Wall )
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


message("octomap include_dirs: " ${OCTOMAP_INCLUDE_DIRS})
message("octomap libraries: " ${OCTOMAP_LIBRARY_DIRS})

catkin_package(
 LIBRARIES active_slam mini_mav
 CATKIN_DEPENDS image_transport sensor_msgs cv_bridge octomap_msgs message_runtime
std_msgs

)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${EIGEN_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

# project libraries
set (PROJECT_LIB
    ${QT_LIBRARIES}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Boost_LIBRARIES}
    ${EIGEN_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
  )



# controller
set(CONTROLLER_HEADERS
  controller/planner/Dstar.h
  controller/planner/spline.h
  controller/planner/controller.h
  controller/stateEstimator/stateestimation.h
  controller/stateEstimator/scale.h
  controller/visualizer/display.h
  controller/mav.h      
  controller/header.h
  controller/planner/actuator.h


)
set(CONTROLLER_SOURCES
  controller/planner/Dstar.cpp
  controller/planner/controller.cpp
  controller/stateEstimator/stateestimation.cpp
  controller/stateEstimator/scale.cpp
  controller/visualizer/display.cpp
  controller/main.cpp
  controller/mav.cpp
  controller/planner/actuator.cpp

)

# build!
 add_executable(motion ${CONTROLLER_HEADERS} ${CONTROLLER_SOURCES})
 target_link_libraries(motion ${PROJECT_LIB})
add_dependencies(motion ${PROJECT_NAME}_gencfg)


#path planner
set(PLANNER_HEADERS
  pathPlanner/planner/Dstar.h
  pathPlanner/planner/spline.h
  pathPlanner/planner/octomap_search.h
  pathPlanner/stateEstimator/stateestimation.h
  pathPlanner/mav.h
  pathPlanner/header.h
  pathPlanner/planner/actuator.h

)
set(PLANNER_SOURCES
  pathPlanner/planner/Dstar.cpp
  pathPlanner/planner/octomap_search.cpp
  pathPlanner/stateEstimator/stateestimation.cpp
  pathPlanner/visualizer/display.cpp
  pathPlanner/main.cpp
  pathPlanner/mav.cpp
  pathPlanner/planner/actuator.cpp
)
# build!
add_executable(planner ${PLANNER_HEADERS} ${PLANNER_SOURCES})
target_link_libraries(planner ${PROJECT_LIB})


# ---------------------------- HOTSPOT SEEKING --------------------------------------------------


#path planner
set(HEXSEARCH_HEADERS
 hexTree/infotree.h
 hexTree/display.h
 hexTree/dpsolver.h
hexTree/hotspot.h

)
set(HEXSEARCH_SOURCES
 hexTree/infotree.cpp
 hexTree/display.cpp
 hexTree/dpsolver.cpp
 hexTree/main.cpp
hexTree/hotspot.cpp
)
# build!
add_executable(seeking ${HEXSEARCH_HEADERS} ${HEXSEARCH_SOURCES})
target_link_libraries(seeking ${PROJECT_LIB})


# ---------------------------- GUI --------------------------------------------------

set(GUI_SOURCE_FILES         
  gui/main.cpp
  gui/mainwindow.cpp
  gui/ros_thread.cpp
  gui/ros_launch.cpp
)

set(GUI_HEADER_FILES
  gui/mainwindow.h      
  gui/ros_thread.h
  gui/ros_launch.h
)

# *.ui
set(GUI_UI_FILES        
  gui/mainwindow.ui 
)

# *.qrc
set(GUI_RESOURCE_FILES        
)


QT4_ADD_RESOURCES(GUI_RESOURCE_FILES_CPP ${GUI_RESOURCE_FILES})
QT4_WRAP_UI(GUI_UI_FILES_HPP ${GUI_UI_FILES})
QT4_WRAP_CPP(GUI_HEADER_FILES_HPP ${GUI_HEADER_FILES})
# include_directories(${CMAKE_CURRENT_BINARY_DIR})

# # build!
add_executable(gui ${GUI_SOURCE_FILES} ${GUI_RESOURCE_FILES_CPP} ${GUI_UI_FILES_HPP} ${GUI_HEADER_FILES_HPP})
target_link_libraries(gui ${PROJECT_LIB}
)
# add_dependencies(quad_map thirdparty ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


