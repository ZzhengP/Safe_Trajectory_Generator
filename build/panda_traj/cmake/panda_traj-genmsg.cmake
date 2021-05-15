# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "panda_traj: 2 messages, 1 services")

set(MSG_I_FLAGS "-Ipanda_traj:/home/zheng/catkin_ws/src/panda_traj/msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/melodic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/melodic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(panda_traj_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" NAME_WE)
add_custom_target(_panda_traj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "panda_traj" "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" ""
)

get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" NAME_WE)
add_custom_target(_panda_traj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "panda_traj" "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" "geometry_msgs/Pose:geometry_msgs/Quaternion:geometry_msgs/Point"
)

get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" NAME_WE)
add_custom_target(_panda_traj_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "panda_traj" "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" "nav_msgs/Path:geometry_msgs/PoseArray:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Point"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panda_traj
)
_generate_msg_cpp(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panda_traj
)

### Generating Services
_generate_srv_cpp(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panda_traj
)

### Generating Module File
_generate_module_cpp(panda_traj
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panda_traj
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(panda_traj_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(panda_traj_generate_messages panda_traj_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" NAME_WE)
add_dependencies(panda_traj_generate_messages_cpp _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_cpp _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_cpp _panda_traj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panda_traj_gencpp)
add_dependencies(panda_traj_gencpp panda_traj_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panda_traj_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panda_traj
)
_generate_msg_eus(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panda_traj
)

### Generating Services
_generate_srv_eus(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panda_traj
)

### Generating Module File
_generate_module_eus(panda_traj
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panda_traj
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(panda_traj_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(panda_traj_generate_messages panda_traj_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" NAME_WE)
add_dependencies(panda_traj_generate_messages_eus _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_eus _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_eus _panda_traj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panda_traj_geneus)
add_dependencies(panda_traj_geneus panda_traj_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panda_traj_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panda_traj
)
_generate_msg_lisp(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panda_traj
)

### Generating Services
_generate_srv_lisp(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panda_traj
)

### Generating Module File
_generate_module_lisp(panda_traj
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panda_traj
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(panda_traj_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(panda_traj_generate_messages panda_traj_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" NAME_WE)
add_dependencies(panda_traj_generate_messages_lisp _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_lisp _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_lisp _panda_traj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panda_traj_genlisp)
add_dependencies(panda_traj_genlisp panda_traj_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panda_traj_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panda_traj
)
_generate_msg_nodejs(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panda_traj
)

### Generating Services
_generate_srv_nodejs(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panda_traj
)

### Generating Module File
_generate_module_nodejs(panda_traj
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panda_traj
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(panda_traj_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(panda_traj_generate_messages panda_traj_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" NAME_WE)
add_dependencies(panda_traj_generate_messages_nodejs _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_nodejs _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_nodejs _panda_traj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panda_traj_gennodejs)
add_dependencies(panda_traj_gennodejs panda_traj_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panda_traj_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj
)
_generate_msg_py(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/nav_msgs/cmake/../msg/Path.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseArray.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj
)

### Generating Services
_generate_srv_py(panda_traj
  "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj
)

### Generating Module File
_generate_module_py(panda_traj
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(panda_traj_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(panda_traj_generate_messages panda_traj_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/srv/UpdateTrajectory.srv" NAME_WE)
add_dependencies(panda_traj_generate_messages_py _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/TrajProperties.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_py _panda_traj_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zheng/catkin_ws/src/panda_traj/msg/PublishTraj.msg" NAME_WE)
add_dependencies(panda_traj_generate_messages_py _panda_traj_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(panda_traj_genpy)
add_dependencies(panda_traj_genpy panda_traj_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS panda_traj_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panda_traj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/panda_traj
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(panda_traj_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(panda_traj_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panda_traj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/panda_traj
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(panda_traj_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(panda_traj_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panda_traj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/panda_traj
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(panda_traj_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(panda_traj_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panda_traj)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/panda_traj
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(panda_traj_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(panda_traj_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/panda_traj
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(panda_traj_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(panda_traj_generate_messages_py nav_msgs_generate_messages_py)
endif()
