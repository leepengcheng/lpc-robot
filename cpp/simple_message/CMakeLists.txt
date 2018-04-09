cmake_minimum_required(VERSION 2.8.12)

project(robot_server)
# set(CMAKE_BUILD_TYPE debug)


# The simple_message library is designed to cross compile on Ubuntu
# and various robot controllers.  This requires conditionally compiling
# certain functions and headers.  The definition below enables compiling
# for a ROS node.
add_definitions(-DLINUXSOCKETS=1)  #use linux sockets for communication
add_definitions(-DLOG=1)  #use linux sockets for communication
add_definitions(-std=c++11)
set(SRC_FILES src/byte_array.cpp
	src/simple_message.cpp
	src/smpl_msg_connection.cpp

	src/socket/simple_socket.cpp
	src/socket/udp_socket.cpp
	src/socket/udp_client.cpp
	src/socket/udp_server.cpp
	src/socket/tcp_socket.cpp
	src/socket/tcp_client.cpp
	src/socket/tcp_server.cpp

	src/message_handler.cpp
	src/message_manager.cpp
	src/ping_handler.cpp
	src/ping_message.cpp
	src/joint_data.cpp
	src/joint_feedback.cpp
	src/joint_traj_pt.cpp
	src/joint_traj_pt_full.cpp
	src/joint_traj.cpp
	src/robot_status.cpp

	src/messages/joint_message.cpp
	src/messages/joint_feedback_message.cpp
	src/messages/joint_traj_pt_message.cpp
	src/messages/joint_traj_pt_full_message.cpp
	src/messages/robot_status_message.cpp
	src/simple_comms_fault_handler.cpp)
					
set(TEST_SRC_FILES test/utest.cpp test/utest_message.cpp   test/server_test.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

# DEFAULT LIBRARY (SAME ENDIAN)
add_library(simple_message ${SRC_FILES})

# ALTERNATIVE LIBRARY (DIFFERENT ENDIAN)
add_library(simple_message_bswap ${SRC_FILES})
set_target_properties(simple_message_bswap PROPERTIES COMPILE_DEFINITIONS "BYTE_SWAPPING")

# ALTERNATIVE LIBRARY (64-bit floats)
add_library(simple_message_float64 ${SRC_FILES})
set_target_properties(simple_message_float64 PROPERTIES COMPILE_DEFINITIONS "FLOAT64")



#服务端
add_executable(motion_server test/motion_server.cpp)
target_link_libraries(motion_server simple_message)

add_executable(state_server test/state_server.cpp)
target_link_libraries(state_server simple_message)

