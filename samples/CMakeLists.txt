cmake_minimum_required(VERSION 3.10)

project(SphereGen)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(glm_PATH "/opt/homebrew/opt/glm")

include_directories("${glm_PATH}/include")

add_executable(SphereGen hemisphere_gen.cpp)
# target_include_directories(SphereGen "${glm_PATH}/include")