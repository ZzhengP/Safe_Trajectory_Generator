
add_executable(trajectory_send trajectory_send.cpp)
target_link_libraries(trajectory_send PRIVATE ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} panda_mpc)


add_executable(test_plane_solver test_plane_solver.cpp)
target_link_libraries(test_plane_solver PRIVATE ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} panda_mpc)

add_executable(tf_broadcast tf_broadcast.cpp)
target_link_libraries(tf_broadcast PRIVATE ${catkin_LIBRARIES} panda_mpc)
