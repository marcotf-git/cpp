# CMakeList.txt : CMake project for CapstoneProjectCmake

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++17)

set(CXX_FLAGS, "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(CapstoneProjectCmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS} src)

find_package(Threads)


add_executable(CapstoneProjectCmake "src/Main.cpp" "src/CylinderObject.cpp" "src/Wall.cpp" "src/Ball.cpp")
string(STRIP ${SDL2_LIBRARIES} SDL2_LIBRARIES)
target_link_libraries(CapstoneProjectCmake ${CMAKE_THREAD_LIBS_INIT} ${SDL2_LIBRARIES} )
