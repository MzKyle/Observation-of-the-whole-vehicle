cmake_minimum_required(VERSION 3.5)

# 设置项目名称
project(MyProject)

# 设置C++标准，这里使用C++17
set(CMAKE_CXX_STANDARD 17)

# 查找OpenCV库
find_package(OpenCV REQUIRED)

# 添加头文件搜索路径
include_directories(/usr/local/include/)

# 添加可执行文件
#add_executable(Mytest test.cpp)

# 链接OpenCV库
#target_link_libraries(Mytest ${OpenCV_LIBS})