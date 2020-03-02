cmake_minimum_required(VERSION 2.8.3)
project(visual_odometry)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation sensor_msgs cv_bridge image_transport)
find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp std_msgs)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(visual_odometry src/visual_odometry.cpp src/callbacks.cpp src/epnp.cpp src/quaternion.cpp)
add_dependencies(visual_odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(visual_odometry ${catkin_LIBRARIES} ${OpenCV_LIBS})
