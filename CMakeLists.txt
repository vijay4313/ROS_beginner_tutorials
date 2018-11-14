cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)


add_compile_options(-std=c++11 -Wall -Wextra)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg std_srvs message_generation tf)

## Flag to run tests automatically
option(CATKIN_ENABLE_TESTING "Flag to run test" ON)

## Declare ROS messages and services
add_service_files(FILES changeText.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

catkin_package(CATKIN_DEPENDS message_runtime)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  #find_package(gtest)
  add_rostest_gtest(rostest1 test/test1.launch test/main.cpp test/test.cpp)
  target_link_libraries(rostest1 ${catkin_LIBRARIES})
endif()
