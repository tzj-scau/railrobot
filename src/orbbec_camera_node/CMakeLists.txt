cmake_minimum_required(VERSION 3.8)
project(orbbec_camera_node)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rail_robot_inspection_msgs REQUIRED)

# 添加共享库
add_library(${PROJECT_NAME} SHARED
  src/orbbec_camera_node.cpp
)

# 设置目标属性
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL"
)

# 链接依赖包
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  sensor_msgs
  rail_robot_inspection_msgs
)

# 注册组件
rclcpp_components_register_nodes(${PROJECT_NAME} "orbbec_camera_node::OrbbecCameraNode")

# 安装目标
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出目标
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 导出依赖
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  rail_robot_inspection_msgs
)

# 导出目标
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()