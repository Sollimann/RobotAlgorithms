cmake_minimum_required(VERSION 3.10)
project(bt_tutorial)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

find_package(BehaviorTreeV3)
find_package(Eigen3)

if(Eigen3_FOUND)
    set(CUSTOM_EIGEN ${EIGEN3_INCLUDE_DIR})
else()
    find_package(Eigen)
    set(CUSTOM_EIGEN ${EIGEN_INCLUDE_DIR})
endif()

include_directories(
        ${CUSTOM_EIGEN}
)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} BT::behaviortree_cpp_v3)