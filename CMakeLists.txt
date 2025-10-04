cmake_minimum_required(VERSION 3.10)
project(uav_trajectoryopt LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debug symbols
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0")

# find packages
find_package(yaml-cpp REQUIRED)

# Add source files
file(GLOB SOURCES src/*.cpp)


# Create executable
add_library(uav_trajectoryopt ${SOURCES})
target_include_directories(uav_trajectoryopt PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(uav_trajectoryopt PRIVATE yaml-cpp)

# Optionally, add more dependencies here
# find_package(OpenCV REQUIRED)
# target_link_libraries(uav_trajectoryopt ${OpenCV_LIBS})

# Install rules (optional)
install(TARGETS uav_trajectoryopt DESTINATION bin)


# example executables 
file(GLOB EXAMPLES examples/*.cpp)
foreach(example_file ${EXAMPLES})
    get_filename_component(example_name ${example_file} NAME_WE)
    add_executable(${example_name} ${example_file})
    target_link_libraries(${example_name} PRIVATE uav_trajectoryopt)
    target_include_directories(${example_name} PRIVATE ${PROJECT_SOURCE_DIR}/include)
endforeach()

# Add this to your CMakeLists.txt to verify
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "C++ Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
