#visp
find_package(VISP REQUIRED visp_core  visp_vs visp_robot)
link_libraries(${VISP_LIBS})
link_libraries("/home/zen/apps/V-REP/libboost_system.so.1.54.0")

add_executable(state_display_tutorial src/state_display_tutorial.cpp)
add_executable(state_visp_tutorial src/state_visp_tutorial.cpp)

target_link_libraries(state_display_tutorial ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )
target_link_libraries(state_visp_tutorial ${catkin_LIBRARIES}  ${Boost_LIBRARIES} )

install(TARGETS state_display_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
