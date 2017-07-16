cmake_minimum_required(VERSION 2.8.3)
project(gazebo_tutorials)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  gazebo_ros 
  std_msgs
  message_generation
)

add_message_files(
  FILES
  inverted_pendulum_states.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
  CATKIN_DEPENDS 
    message_runtime
    std_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(simple_world_plugin src/simple_world_plugin.cpp)
add_library(model_measure src/model_measure.cc)
add_executable(gazebo_tutorial_listener src/gazebo_tutorial_listener.cpp)
add_dependencies(gazebo_tutorial_listener gazebo_tutorials_generate_messages_cpp)
target_link_libraries(simple_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
target_link_libraries(model_measure ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(gazebo_tutorial_listener ${catkin_LIBRARIES})






