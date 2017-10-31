cmake_minimum_required(VERSION 2.8.3)
project(mpc_controller)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS cmake_modules cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp roslint sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen REQUIRED)

find_package(Ceres REQUIRED)
find_package(ACADO REQUIRED)
message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")


add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs
  DEPENDS Boost CERES ACADO
  INCLUDE_DIRS include
  LIBRARIES inv_calculation kinematic_computation
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${ACADO_INCLUDE_DIRS})

add_library(inv_calculation src/inverse_jacobian_calculation/inverse_jacobian_calculation.cpp)
add_dependencies(inv_calculation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(inv_calculation ${catkin_LIBRARIES})

add_library(kinematic_computation src/Kinematics/kinematics.cpp)
add_dependencies(kinematic_computation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematic_computation ${catkin_LIBRARIES})

FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

### Test Case ####
add_executable(kinematics_test test/Kinematics/kinematics_test.cpp)
add_dependencies(kinematics_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematics_test inv_calculation kinematic_computation ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${libacado})

### INSTALL ###
install(TARGETS inv_calculation kinematic_computation
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
