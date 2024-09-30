cmake_minimum_required(VERSION 3.8)
project(allegro_hand_keyboards)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(allegro_hand_driver REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(
  include
)

add_executable(allegro_hand_keyboard src/allegro_hand_keyboard.cpp)

target_link_libraries(allegro_hand_keyboard

)

ament_target_dependencies(allegro_hand_keyboard
  rclcpp
  std_msgs
  allegro_hand_driver
  yaml_cpp_vendor
)

install(TARGETS
  allegro_hand_keyboard
  # allegro_node_torque
  # allegro_node_velsat
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
