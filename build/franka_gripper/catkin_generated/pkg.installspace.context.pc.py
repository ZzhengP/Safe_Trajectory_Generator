# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/zheng/libfranka/include".split(';') if "${prefix}/include;/home/zheng/libfranka/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;message_runtime;control_msgs;actionlib;sensor_msgs;xmlrpcpp;actionlib_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lfranka_gripper;/home/zheng/libfranka/build/libfranka.so.0.8.0".split(';') if "-lfranka_gripper;/home/zheng/libfranka/build/libfranka.so.0.8.0" != "" else []
PROJECT_NAME = "franka_gripper"
PROJECT_SPACE_DIR = "/home/zheng/catkin_ws/install"
PROJECT_VERSION = "0.7.1"
