cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS " -O3 -g -std=gnu++0x -msse -msse2 -msse3 -Wall -Wextra -Winit-self -Wpointer-arith -Wno-sign-compare -Wcast-qual -Wno-unused-parameter -Wno-uninitialized -Wno-missing-field-initializers -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-promo ${CMAKE_CXX_FLAGS}")
set(HANDCLASS_ROOT /home/javier/hand/handclass/branches/refactoring)
include_directories(${HANDCLASS_ROOT}/include /usr/local/include/eigen3)
link_directories(${HANDCLASS_ROOT}/build/src)

add_executable(testhand testhand.cc)
target_link_libraries(testhand buolascene buolagui buolafsgui boost_system)

add_executable(renderhand renderhand.cc)
target_link_libraries(renderhand buolascene buolagui buolafsgui buolacv boost_system boost_filesystem
                      opencv_highgui sqlite3 handclass)
