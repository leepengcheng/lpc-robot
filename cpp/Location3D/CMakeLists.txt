cmake_minimum_required(VERSION 2.8)
project(Location3D)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_BUILD_TYPE Release)
add_definitions(-std=c++11)
link_directories(${CMAKE_SOURCE_DIR}/lib)

# ##########Qt#################
set(QT_UIC_EXECUTABLE C:/Programs/Qt/5.9.6/msvc2015_64/bin/uic.exe)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
file(GLOB SRC_FILES "./*.cpp")
file(GLOB HEADER_FILES "./*.h")
file(GLOB UI_FILES "./*.ui")
qt_wrap_ui(WRAP_FILES ${UI_FILES})

include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories(${Qt5Widgets_INCLUDE_DIRS})
include_directories(${Qt5Gui_INCLUDE_DIRS})
link_libraries(Qt5::Core)
link_libraries(Qt5::Widgets)
link_libraries(Qt5::Gui)


# #####Halcon#########################
set(HALCONROOT C:/Programs/Halcon)
link_directories(${HALCONROOT}/lib/x64-win64)
link_libraries(halconcpp.lib halcon.lib)
include_directories(${HALCONROOT}/include  ${HALCONROOT}/include/halconcpp)
# # file(COPY ${HALCONROOT}/bin/x64-win64/halcon.dll DESTINATION  ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
# # file(COPY ${HALCONROOT}/bin/x64-win64/halconcpp.dll DESTINATION ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})

# ###############ZeroMq##############
set(BOOST_INCLUDE_DIRS "D:/library/boost")
set(B0_INCLUDE_DIRS "./include/bluezero")
set(BOOST_LIB_DIRS "${BOOST_INCLUDE_DIRS}/lib64-msvc-14.0")
include_directories(${BOOST_INCLUDE_DIRS})
include_directories(${B0_INCLUDE_DIRS})
link_directories(${BOOST_LIB_DIRS})
link_libraries("./lib/b0.lib")


# ##############MsgPack##########
# set(RPCLIB_INCLUDE_DIR include)
# set(RPCLIB_COMPILE_DEFINITIONS
#     "ASIO_STANDALONE"
#     "RPCLIB_ASIO=clmdep_asio"
#     "RPCLIB_FMT=clmdep_fmt"
#     "RPCLIB_MSGPACK=clmdep_msgpack"
# )
# set(RPCLIB_LIBS        rpc.lib)
# set(RPCLIB_EXTRA_FLAGS "/EHsc")
# set(RPCLIB_EXTRA_FLAGS_DEBUG "/Zi")
# include_directories(${RPCLIB_INCLUDE_DIR})

# # add_executable(calculator_server calculator_server.cc)
# # target_link_libraries(calculator_server ${RPCLIB_LIBS} ${OpenCV_LIBS} )
# # set_target_properties(
# #         calculator_server
# #         PROPERTIES
# #         CXX_STANDARD 11
# #         COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${RPCLIB_EXTRA_FLAGS}")
# # target_compile_definitions(calculator_server PUBLIC ${RPCLIB_COMPILE_DEFINITIONS})

# # add_executable(calculator_client calculator_client.cc)
# # target_link_libraries(calculator_client ${RPCLIB_LIBS} ${OpenCV_LIBS} )
# # set_target_properties(
# #         calculator_client
# #         PROPERTIES
# #         CXX_STANDARD 14
# #         COMPILE_FLAGS "${CMAKE_CXX_FLAGS} ${RPCLIB_EXTRA_FLAGS}")
# # target_compile_definitions(calculator_client PUBLIC ${RPCLIB_COMPILE_DEFINITIONS})

add_executable(${PROJECT_NAME} WIN32 ${SRC_FILES} ${HEADER_FILES} ${WRAP_FILES})
