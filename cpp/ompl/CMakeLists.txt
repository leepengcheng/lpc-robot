cmake_minimum_required(VERSION 2.8.3)
project(ompl_demos)
set(CMAKE_CXX_FLAGS "-std=c++11")

set(CMAKE_BUILD_TYPE debug)
option(USE_OMPL_DEB "use ompl deb package" OFF)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Boost REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(${Boost_INCLUDE_DIR})
link_libraries(${Boost_FILESYSTEM_LIBRARY}  ${Boost_SYSTEM_LIBRARY}  ${Boost_PROGRAM_OPTIONS_LIBRARY}  ${Boost_SERIALIZATION_LIBRARY})

if(USE_OMPL_DEB)
    find_package(OMPL REQUIRED)
else()
    set(OMPL_DIR "/media/lee/download/SLAM/ompl-master")
    set(OMPL_INCLUDE_DIRS ${OMPL_DIR}/src)
    set(OMPL_LIBRARIES ${OMPL_DIR}/debug/lib/libompl.so)
endif()
include_directories(${OMPL_INCLUDE_DIRS})
link_libraries(${OMPL_LIBRARIES})




macro(add_ompl_exe file_cpp)
    get_filename_component(target ${file_cpp} NAME_WE)
    add_executable(${target}  ${file_cpp})
endmacro()


add_ompl_exe(src/RigidBodyPlanning.cpp)
add_ompl_exe(src/RigidBodyPlanningWithIK.cpp)
add_ompl_exe(src/RigidBodyPlanningWithControls.cpp)
add_ompl_exe(src/RigidBodyPlanningWithIntegrationAndControls.cpp)
add_ompl_exe(src/RigidBodyPlanningWithODESolverAndControls.cpp)
add_ompl_exe(src/StateSampling.cpp)
add_ompl_exe(src/GeometricCarPlanning.cpp)
add_ompl_exe(src/Point2DPlanning.cpp)
add_ompl_exe(src/HybridSystemPlanning.cpp)
add_ompl_exe(src/KinematicChainBenchmark.cpp)
add_ompl_exe(src/HypercubeBenchmark.cpp)
add_ompl_exe(src/PlannerData.cpp)
add_ompl_exe(src/ThunderLightning.cpp)
add_ompl_exe(src/OptimalPlanning.cpp)
add_ompl_exe(src/PlannerProgressProperties.cpp)
add_ompl_exe(src/CForestCircleGridBenchmark.cpp)
add_ompl_exe(src/Diagonal.cpp)


#VFRRT
add_ompl_exe(src/VFRRT/VectorFieldConservative.cpp)
add_ompl_exe(src/VFRRT/VectorFieldNonconservative.cpp)

#constraint
add_ompl_exe(src/constraint/ConstrainedPlanningSphere.cpp)
add_ompl_exe(src/constraint/ConstrainedPlanningTorus.cpp)
add_ompl_exe(src/constraint/ConstrainedPlanningImplicitChain.cpp)
add_ompl_exe(src/constraint/ConstrainedPlanningImplicitParallel.cpp)
add_ompl_exe(src/constraint/ConstrainedPlanningKinematicChain.cpp)

#Koules
aux_source_directory(src/Koules Koules_SRC)
add_executable(demo_Koules ${Koules_SRC})


if (OMPL_EXTENSION_OPENDE)
    add_ompl_exe(src/OpenDERigidBodyPlanning.cpp)
endif()

if (OMPL_EXTENSION_TRIANGLE)
    add_ompl_exe(src/TriangulationDemo.cpp)
    add_ompl_exe(src/LTLWithTriangulation)
endif()

