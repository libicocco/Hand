cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS " -O0 -g -std=gnu++0x -msse -msse2 -msse3 -Wall -Wextra -Winit-self -Wpointer-arith -Wno-sign-compare -Wcast-qual -Wno-unused-parameter -Wno-uninitialized -Wno-missing-field-initializers -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-promo ${CMAKE_CXX_FLAGS}")
include_directories(../handclass/branches/refactoring/include /usr/local/include/eigen3)

add_executable(testhand testhand.cc)
target_link_libraries(testhand buolascene buolagui buolafsgui boost_system)

add_executable(renderhand renderhand.cc)
target_link_libraries(renderhand buolascene buolagui buolafsgui buolacv boost_system opencv_highgui)

add_executable(isolated isolated.cc)
target_link_libraries(isolated buolascene buolagui buolafsgui boost_system)

