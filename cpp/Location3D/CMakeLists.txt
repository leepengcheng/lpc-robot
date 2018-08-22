cmake_minimum_required(VERSION 2.8.3)
project(ZeroHalcon)

set(CMAKE_BUILD_TYPE Release)
set(HALCONROOT C:/Programs/Halcon)
add_definitions(-std=c++11)

link_directories(${HALCONROOT}/lib/x64-win64)
link_libraries(halconcpp.lib halcon.lib)

include_directories(${HALCONROOT}/include  ${HALCONROOT}/include/halconcpp)
add_executable(ZeroHalcon main.cpp)


file(COPY ${HALCONROOT}/bin/x64-win64/halcon.dll DESTINATION  ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
file(COPY ${HALCONROOT}/bin/x64-win64/halconcpp.dll DESTINATION ${PROJECT_BINARY_DIR}/${CMAKE_BUILD_TYPE})
