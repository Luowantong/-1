cmake_minimum_required(VERSION 3.20)
project(task3)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#设置c++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(yaml-cpp REQUIRED)
#包含头文件目录
include_directories(include)

#查找src目录下所有cpp文件
file(GLOB_RECURSE SOURCE "src/*.cpp")
#生成可执行文件
add_executable(task3${SOURCE})
target_link_libraries(task3 yaml-cpp)