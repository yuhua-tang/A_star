cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(A_Star)

find_package(Protobuf 3.0.0 REQUIRED)
find_package(PythonLibs 2.7)
find_package(Eigen3 REQUIRED)

#添加debug信息，使得生成的可执行文件可以调试
#set(CMAKE_BUILD_TYPE DEBUG)
#使用通配符添加多个源文件
#file(GLOB SRC_LIST "src/*.c")

#编译选项
add_compile_options(-std=c++11)


# 头文件路径
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${EIGEN3_INCLUDE_DIRS})


# #链接库
# link_libraries(event)

# 生成可执行的文件
add_executable(${PROJECT_NAME} src/A_star.cpp
                src/main.cpp src/graph.cpp)

target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    ${PYTHON_INCLUDE_DIRS})

