
add_executable(trajectory_send trajectory_send.cpp)
target_link_libraries(trajectory_send PRIVATE ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} panda_mpc)
