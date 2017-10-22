# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_localization: 0 messages, 2 services")

set(MSG_I_FLAGS "-Igeographic_msgs:/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Iuuid_msgs:/opt/ros/indigo/share/uuid_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_localization_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv" NAME_WE)
add_custom_target(_robot_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_localization" "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv" "geometry_msgs/Quaternion:geographic_msgs/GeoPose:geographic_msgs/GeoPoint"
)

get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv" NAME_WE)
add_custom_target(_robot_localization_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_localization" "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv" "geometry_msgs/Point:geometry_msgs/PoseWithCovariance:geometry_msgs/PoseWithCovarianceStamped:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Pose"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(robot_localization
  "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg/GeoPose.msg;/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
)
_generate_srv_cpp(robot_localization
  "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_cpp(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_cpp _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_cpp _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_gencpp)
add_dependencies(robot_localization_gencpp robot_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(robot_localization
  "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg/GeoPose.msg;/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
)
_generate_srv_lisp(robot_localization
  "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_lisp(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_lisp _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_lisp _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_genlisp)
add_dependencies(robot_localization_genlisp robot_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(robot_localization
  "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg/GeoPose.msg;/home/hl/helei_ws/src/geographic_info-master/geographic_msgs/msg/GeoPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
)
_generate_srv_py(robot_localization
  "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseWithCovarianceStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
)

### Generating Module File
_generate_module_py(robot_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_localization_generate_messages robot_localization_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetDatum.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_py _robot_localization_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hl/helei_ws/src/robot_localization-indigo-devel/srv/SetPose.srv" NAME_WE)
add_dependencies(robot_localization_generate_messages_py _robot_localization_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_localization_genpy)
add_dependencies(robot_localization_genpy robot_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_localization_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_cpp)
  add_dependencies(robot_localization_generate_messages_cpp geographic_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robot_localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_localization_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_lisp)
  add_dependencies(robot_localization_generate_messages_lisp geographic_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robot_localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_localization_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geographic_msgs_generate_messages_py)
  add_dependencies(robot_localization_generate_messages_py geographic_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robot_localization_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_localization_generate_messages_py std_msgs_generate_messages_py)
endif()
