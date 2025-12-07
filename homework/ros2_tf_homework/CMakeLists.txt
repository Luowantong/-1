cmake_minimum_required(VERSION 3.8)
project(ros2_tf_homework)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(homework_interface REQUIRED)
find_package(tf2 REQUIRED)

# 只需要一个可执行文件（main.cpp 使用了头文件中的类）
add_executable(ros2_tf_homework src/main.cpp)

# 添加包含目录
target_include_directories(ros2_tf_homework PUBLIC
  include
  ${ament_INCLUDE_DIRS}
)

# 链接库
ament_target_dependencies(ros2_tf_homework
  rclcpp
  tf2_ros
  geometry_msgs
  std_srvs
  rclcpp_action
  homework_interface
)

# 安装可执行文件
install(TARGETS ros2_tf_homework
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（整个目录）
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()