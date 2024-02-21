cmake_minimum_required(VERSION 3.8)
project(pseudo_LiDAR_from_pc_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
#use PCL
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
#use OpenCV
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include ${pseudo_LiDAR_from_pc_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${pseudo_LiDAR_from_pc_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})
add_definitions(${pseudo_LiDAR_from_pc_DEFINITIONS} ${PCL_DEFINITIONS} ${OpenCV_DEFINITIONS})

add_executable(pseudo_LiDAR_from_pc src/pseudo_LIDAR_from_pc.cpp)
ament_target_dependencies(pseudo_LiDAR_from_pc 
rclcpp 
sensor_msgs
pcl_msgs
pcl_conversions
OpenCV
cv_bridge
)


target_link_libraries(pseudo_LiDAR_from_pc ${pseudo_LiDAR_from_pc_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS
  pseudo_LiDAR_from_pc
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
