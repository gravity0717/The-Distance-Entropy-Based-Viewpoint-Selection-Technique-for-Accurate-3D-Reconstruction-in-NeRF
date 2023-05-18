cmake_minimum_required(VERSION 3.10)

project(SphereGen)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(glm REQUIRED)

add_executable(SphereGen hemisphere_gen.cpp)

target_link_libraries(SphereGen glm)
