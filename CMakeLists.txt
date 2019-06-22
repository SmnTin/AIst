cmake_minimum_required(VERSION 2.8.3)
project(aist)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslaunch
  message_generation
  image_transport
  cv_bridge
)

roslaunch_add_file_check(launch)

find_package( OpenCV 3.4.6 REQUIRED )
#find_package( OpenCV 3.4.1 REQUIRED )

add_message_files(DIRECTORY msg FILES LineInfo.msg Motors.msg LineControl.msg TrafficSign.msg TrafficSignsInfo.msg TrafficLightsInfo.msg)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aist
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(line_node2 src/LineNode/LineNode.cpp src/LineNode/LineDetector.h src/LineNode/LineDetector.cpp src/LineNode/Runner.h src/LineNode/Runner.cpp)
target_link_libraries(line_node2 ${catkin_LIBRARIES})
target_link_libraries(line_node2 ${OpenCV_LIBS} )
add_dependencies(line_node2 aist_generate_messages_cpp)

add_executable(traffic_signs_node src/TrafficSignsNode/TrafficSignsNode.cpp src/TrafficSignsNode/Runner.h src/TrafficSignsNode/Runner.cpp src/TrafficSignsNode/TrafficSignsDetector.h src/TrafficSignsNode/TrafficSignsDetector.cpp)
target_link_libraries(traffic_signs_node ${catkin_LIBRARIES})
target_link_libraries(traffic_signs_node ${OpenCV_LIBS} )
add_dependencies(traffic_signs_node aist_generate_messages_cpp)

add_executable(traffic_lights_node src/TrafficLightsNode/TrafficLightsNode.cpp src/TrafficLightsNode/Runner.h src/TrafficLightsNode/Runner.cpp src/TrafficLightsNode/TrafficLightsDetector.h src/TrafficLightsNode/TrafficLightsDetector.cpp)
target_link_libraries(traffic_lights_node ${catkin_LIBRARIES})
target_link_libraries(traffic_lights_node ${OpenCV_LIBS} )
add_dependencies(traffic_lights_node aist_generate_messages_cpp)

add_executable(traffic_lights_nn_node src/TrafficLightsNNNode/TrafficLightsNNNode.cpp src/TrafficLightsNNNode/Runner.h src/TrafficLightsNNNode/Runner.cpp src/TrafficLightsNNNode/TrafficLightsDetector.h src/TrafficLightsNNNode/TrafficLightsDetector.cpp src/TrafficLightsNNNode/net.hpp src/TrafficLightsNNNode/net.cpp src/TrafficLightsNNNode/activation.hpp src/TrafficLightsNNNode/trainDataCollection.cpp src/TrafficLightsNNNode/trainDataCollection.hpp src/TrafficLightsNNNode/signs.hpp)
target_link_libraries(traffic_lights_nn_node ${catkin_LIBRARIES})
target_link_libraries(traffic_lights_nn_node ${OpenCV_LIBS} )
add_dependencies(traffic_lights_nn_node aist_generate_messages_cpp)

add_executable(movement_node src/MovementNode/MovementNode.cpp src/MovementNode/JHPWMPCA9685.h src/MovementNode/JHPWMPCA9685.cpp)
target_link_libraries(movement_node ${catkin_LIBRARIES})
target_link_libraries(movement_node ${OpenCV_LIBS} )
add_dependencies(movement_node aist_generate_messages_cpp)

add_executable(camera_node src/CameraNode/CameraNode.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})
target_link_libraries(camera_node ${OpenCV_LIBS} )
add_dependencies(camera_node aist_generate_messages_cpp)

add_executable(core_node src/CoreNode/CoreNode.cpp src/CoreNode/Core.h src/CoreNode/Core.cpp src/CoreNode/StateMachine.h src/CoreNode/PID.h src/CoreNode/PID.cpp src/CoreNode/States/Common/TurningState.h src/CoreNode/States/Common/RedSignalState.h src/CoreNode/Data.h src/CoreNode/States/Qualification/QualificationInitState.h src/CoreNode/States/Qualification/QualificationState.h src/CoreNode/States/Common/SlowDownState.h src/CoreNode/States/KnownPath/KnownPathState.h src/CoreNode/States/KnownPath/KnownPathInitState.h src/CoreNode/States/UnknownPath/UnknownPathState.h src/CoreNode/States/UnknownPath/UnknownPathInitState.h src/CoreNode/States/Common/reactToSlowdown.h src/CoreNode/States/ProblematicRoute/ProblematicRouteState.h src/CoreNode/States/ProblematicRoute/ProblematicRouteInitState.h src/CoreNode/States/ProblematicRoute/CurvyLineState.h src/CoreNode/States/Qualification/TurningState.h src/CoreNode/States/ProblematicRoute/ChangeLineState.h)
target_link_libraries(core_node ${catkin_LIBRARIES})
target_link_libraries(core_node ${OpenCV_LIBS} )
add_dependencies(core_node aist_generate_messages_cpp)