cmake_minimum_required(VERSION 3.22)
project(allegro_hand_moveit)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.launch.py"
  PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
