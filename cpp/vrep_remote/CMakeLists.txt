cmake_minimum_required(VERSION 2.8)
project(vrepRemoteCpp)



set(CMAKE_BUILD_TYPE "debug")
# 添加c++ 11标准支持
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -Wall -Wno-unused-parameter -Wno-strict-aliasing -Wno-empty-body-Wno-write-strings -Wno-unused-but-set-variable -Wno-unused-local-typedefs -Wno-narrowing")

set(CMAKE_C_FLAGS "-O3 -Wall -Wno-strict-aliasing -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-unused-local-typedefs")

add_definitions(-DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DLIN_VREP)

#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

# 添加vrep include
if(DEFINED ENV{VREP_ROOT})
    set(VREP_REMOTE_DIR  $ENV{VREP_ROOT}/programming/remoteApi)
    set(VREP_INCLUDE_DIRS ${VREP_REMOTE_DIR} $ENV{VREP_ROOT}/programming/include)
else()
    message(FATAL_ERROR  "NO DEFINED VREP ROOT DIR")
endif()


#set(VREP_REMOTE_SRC
#    ${VREP_REMOTE_DIR}/extApiPlatform.c
#    ${VREP_REMOTE_DIR}/extApi.c
#)

# VREP LIBS
set(VREP_LIBS $ENV{VREP_ROOT}/programming/remoteApiBindings/lib/lib/64Bit/remoteApi.so)
LINK_LIBRARIES(${VREP_LIBS})

set(demos 
vrepIbvsAruco_1.cpp
vrepIbvsAruco_2.cpp)




find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(VISP REQUIRED visp_core  visp_vs visp_robot)


include_directories(${VREP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# add_executable(vrepHalcon vrepHalcon.cpp)

foreach(file_cpp ${demos})
    get_filename_component(target ${file_cpp} NAME_WE)
    add_executable(${target}  ${file_cpp})
    target_link_libraries(${target}  ${OpenCV_LIBS} ${VISP_LIBS} pthread)
endforeach()
