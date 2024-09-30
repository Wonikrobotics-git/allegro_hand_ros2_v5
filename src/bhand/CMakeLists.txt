cmake_minimum_required(VERSION 3.5)
project(bhand)

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
# Set the output directory for libraries
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

###########
## Build ##
###########

include_directories(include)

# Add library
add_library(BHand SHARED IMPORTED)
set_target_properties(BHand PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/lib/libBHand.so
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/include/bhand
)

# If you have any executables or other targets, add them here

#############
## Install ##
#############

## Mark header files for installation
install(DIRECTORY include/bhand/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libBHand.so
  DESTINATION lib
)

ament_export_include_directories(include/bhand)
ament_export_libraries(BHand)

###############
## ament specific configuration ##
###############
ament_package()
