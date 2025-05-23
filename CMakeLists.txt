cmake_minimum_required(VERSION 3.5)
project(autonomy_utils)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
        rclcpp
        std_msgs
        std_srvs
        sensor_msgs
        geometry_msgs
	      nav_msgs
        rclcpp_lifecycle
        example_interfaces
        Eigen3
        jsoncpp
        OpenCV
        cv_bridge
        PCL
        pcl_conversions
        rosbag2
        rosbag2_cpp
        rosbag2_storage
        sensor_msgs
        stereo_msgs
        tf2
        tf2_geometry_msgs
        tf2_sensor_msgs
        tf2_ros
        visualization_msgs
      )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)

foreach(Dependency IN ITEMS ${dependencies})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
	include
  	)



######################################BEGIN DOCUMENTATION##################################
# Enable Doxygen documentation generation
find_package(Doxygen REQUIRED)

set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

add_custom_target(doc
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
)
# Install generated documentation
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/docs
    DESTINATION share/${PROJECT_NAME}/
    OPTIONAL
)

######################################END DOCUMENTATION##################################


###################################AUTNOMY_UTILS_LIB#####################################
file(GLOB utils_lib_files
src/autonomy_utils/lib/*.cpp
)

add_library(autonomy_utils_lib 
  ${utils_lib_files}
)
target_link_libraries(autonomy_utils_lib)
ament_target_dependencies(autonomy_utils_lib ${dependencies})
ament_export_targets(autonomy_utils_libTargets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(autonomy_utils_lib)

###################################KEYBOARD_NODE#####################################

add_executable(keyboard_node src/nodes/keyboard_node.cpp)
target_link_libraries(keyboard_node autonomy_utils_lib)
ament_target_dependencies(keyboard_node ${dependencies})

add_executable(imu_to_rpy_node src/nodes/imu_to_rpy_node.cpp)
target_link_libraries(imu_to_rpy_node autonomy_utils_lib)
ament_target_dependencies(imu_to_rpy_node ${dependencies})


install(DIRECTORY include/
        DESTINATION include/
    )
    
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}
    )
install(
  TARGETS autonomy_utils_lib
  EXPORT autonomy_utils_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
  keyboard_node
  imu_to_rpy_node
  DESTINATION lib/${PROJECT_NAME})

enable_testing()
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test test/All_test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_test autonomy_utils_lib)
  ament_target_dependencies(${PROJECT_NAME}_test
    ${dependencies}
  )
  #target_link_libraries(${PROJECT_NAME}_test name_of_local_library)
endif()


ament_export_include_directories(include)
ament_export_libraries(autonomy_utils_lib)
ament_export_dependencies(${dependencies})
ament_package()
