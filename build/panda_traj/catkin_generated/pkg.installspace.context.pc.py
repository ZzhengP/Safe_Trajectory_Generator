# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3".split(';') if "${prefix}/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;geometry_msgs;nav_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpanda_traj;/opt/ros/melodic/lib/liborocos-kdl.so.1.4.0".split(';') if "-lpanda_traj;/opt/ros/melodic/lib/liborocos-kdl.so.1.4.0" != "" else []
PROJECT_NAME = "panda_traj"
PROJECT_SPACE_DIR = "/home/zheng/catkin_ws/install"
PROJECT_VERSION = "0.0.0"
