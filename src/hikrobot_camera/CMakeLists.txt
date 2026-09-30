cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

# ---------- 编译选项 ----------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------- ROS 2 依赖 ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---------- 海康 MVS SDK ----------
# 如果 SDK 装在 /opt/MVS，可以直接用；如果不是，改这里或在编译时通过 -DMVS_ROOT=... 覆盖
set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK install path")
set(MVS_INCLUDE_DIR "${MVS_ROOT}/include")
set(MVS_LIB_DIR "${MVS_ROOT}/lib/64")

if(NOT EXISTS "${MVS_INCLUDE_DIR}/MvCameraControl.h")
  message(FATAL_ERROR
    "未找到 MvCameraControl.h (${MVS_INCLUDE_DIR})。请安装海康 MVS SDK 或设置 -DMVS_ROOT=<path>")
endif()

# ---------- 可执行文件 ----------
add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
)

target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE
  include
  ${MVS_INCLUDE_DIR}
)

ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

# 链接海康 MVS 库
target_link_directories(camera_node PRIVATE ${MVS_LIB_DIR})
target_link_libraries(camera_node MvCameraControl)

# ---------- 安装 ----------
install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()