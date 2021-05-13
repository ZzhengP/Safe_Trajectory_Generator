
add_executable(qptest qptest.cpp)
target_link_libraries(qptest PRIVATE ${catkin_LIBRARIES} ${qpOASES_LIBRARIES} panda_mpc)
