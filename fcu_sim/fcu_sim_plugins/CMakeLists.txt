cmake_minimum_required(VERSION 2.8.3)
project(fcu_sim_plugins)

add_definitions(-std=c++11)
add_definitions(-std=c99)

# To enable assertions when compiled in release mode.
add_definitions(-DROS_ASSERT_ENABLED)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  geometry_msgs
  fcu_sim
  rosbag
  roscpp
  std_srvs
  tf
  fcu_common
)

find_package(Eigen REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include/ROSflight_SIL)

pkg_check_modules(OGRE OGRE)
pkg_check_modules(OGRE-Terrain OGRE-Terrain)
pkg_check_modules(OGRE-Paging OGRE-Paging)
include_directories(
    ${OGRE_INCLUDE_DIRS}
    ${OGRE-Terrain_INCLUDE_DIRS}
    ${OGRE-Paging_INCLUDE_DIRS}
)


include_directories(${Eigen_INCLUDE_DIRS})
catkin_package(
  INCLUDE_DIRS include ${eigen_INCLUDE_DIRS}
  CATKIN_DEPENDS cv_bridge geometry_msgs fcu_sim rosbag roscpp
                 fcu_sim std_srvs tf relative_nav relative_nav
  DEPENDS eigen gazebo opencv fcu_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(fcu_sim_ROSflight_sil_plugin
#  src/gazebo_ROSflight_sil.cpp
#  include/fcu_sim_plugins/gazebo_ROSflight_sil.h
#  include/ROSflight/src/estimator.c
#  include/ROSflight/src/mixer.c
#  include/ROSflight/src/controller.c
#  include/ROSflight_SIL/sensors.c
#  include/ROSflight_SIL/param.c
#  include/ROSflight_SIL/mode.c
#  include/ROSflight_SIL/mux.c
#  include/ROSflight_SIL/breezystm32/breezystm32.c
#  include/ROSflight_SIL/turbotrig/turbotrig.c
#  include/ROSflight_SIL/turbotrig/turbovec.c)
#target_link_libraries(fcu_sim_ROSflight_sil_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#add_dependencies(fcu_sim_ROSflight_sil_plugin ${catkin_EXPORTED_TARGETS})
#set_property( TARGET fcu_sim_ROSflight_sil_plugin APPEND_STRING PROPERTY COMPILE_FLAGS -w )

add_library(fcu_sim_aircraft_forces_and_moments_plugin
  src/gazebo_aircraft_forces_and_moments.cpp
  include/fcu_sim_plugins/gazebo_aircraft_forces_and_moments.h)
target_link_libraries(fcu_sim_aircraft_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_aircraft_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_multirotor_forces_and_moments_plugin
  src/gazebo_multirotor_forces_and_moments.cpp
  include/fcu_sim_plugins/gazebo_multirotor_forces_and_moments.h)
target_link_libraries(fcu_sim_multirotor_forces_and_moments_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_multirotor_forces_and_moments_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_aircraft_truth_plugin
  src/gazebo_aircraft_truth.cpp
  include/fcu_sim_plugins/gazebo_aircraft_truth.h)
target_link_libraries(fcu_sim_aircraft_truth_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_aircraft_truth_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_odometry_plugin
  src/gazebo_odometry_plugin.cpp
  include/fcu_sim_plugins/gazebo_odometry_plugin.h)
target_link_libraries(fcu_sim_odometry_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(fcu_sim_odometry_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_imu_plugin
  src/gazebo_imu_plugin.cpp
  include/fcu_sim_plugins/gazebo_imu_plugin.h)
target_link_libraries(fcu_sim_imu_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_imu_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_altimeter_plugin
  src/gazebo_altimeter_plugin.cpp
  include/fcu_sim_plugins/gazebo_altimeter_plugin.h)
target_link_libraries(fcu_sim_altimeter_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_altimeter_plugin ${catkin_EXPORTED_TARGETS})

# add_library(fcu_sim_wind_plugin
#   src/gazebo_wind_plugin.cpp
#   include/fcu_sim_plugins/gazebo_wind_plugin.h)
# target_link_libraries(fcu_sim_wind_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(fcu_sim_wind_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_airspeed_plugin
  src/gazebo_airspeed_plugin.cpp
  include/fcu_sim_plugins/gazebo_airspeed_plugin.h)
target_link_libraries(fcu_sim_airspeed_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_airspeed_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_GPS_plugin
  src/gazebo_GPS_plugin.cpp
  include/fcu_sim_plugins/gazebo_GPS_plugin.h)
target_link_libraries(fcu_sim_GPS_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_GPS_plugin ${catkin_EXPORTED_TARGETS})

# add_library(fcu_sim_gimbal_plugin
#   src/gazebo_gimbal_plugin.cpp
#   include/fcu_sim_plugins/gazebo_gimbal_plugin.h)
# target_link_libraries(fcu_sim_gimbal_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
# add_dependencies(fcu_sim_gimbal_plugin ${catkin_EXPORTED_TARGETS})

add_library(fcu_sim_fleet_visual_plugin
  src/gazebo_fleet_visual.cpp
  include/fcu_sim_plugins/gazebo_fleet_visual.h)
target_link_libraries(fcu_sim_fleet_visual_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(fcu_sim_fleet_visual_plugin ${catkin_EXPORTED_TARGETS})

install(
  TARGETS
    fcu_sim_odometry_plugin
    fcu_sim_imu_plugin
    fcu_sim_altimeter_plugin
    # fcu_sim_wind_plugin
    # fcu_sim_gimbal_plugin
    fcu_sim_GPS_plugin
    fcu_sim_airspeed_plugin
#    fcu_sim_ROSflight_sil_plugin
    fcu_sim_multirotor_forces_and_moments_plugin
    fcu_sim_aircraft_forces_and_moments_plugin
    fcu_sim_fleet_visual_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
