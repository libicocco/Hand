cmake_minimum_required(VERSION 2.6)
enable_testing()
cmake_policy(SET CMP0014 OLD)
set (CMAKE_BUILD_TYPE DEBUG)
#set (CMAKE_BUILD_TYPE RELWITHDEBINFO)
#set (CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules")
#set(CMAKE_CXX_FLAGS " -O0 -g -std=c++0x -msse -msse2 -msse3 -Wall -Wextra -Winit-self -Wpointer-arith -Wno-sign-compare -Wcast-qual -Wno-unused-parameter -Wno-uninitialized -Wno-missing-field-initializers -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-promo ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS " -O3 -g -std=gnu++0x -msse -msse2 -msse3 -Wall -Wextra -Winit-self -Wpointer-arith -Wno-sign-compare -Wcast-qual -Wno-unused-parameter -Wno-uninitialized -Wno-missing-field-initializers -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-promo ${CMAKE_CXX_FLAGS}")
set (SOURCES_DIR_LSHKIT ${CMAKE_SOURCE_DIR}/3rdparty/lshkit-0.2.1)
set (SOURCES_DIR_ANN ${CMAKE_SOURCE_DIR}/3rdparty/ann_1.1.2)
set (SOURCES_DIR_FIGTREE ${CMAKE_SOURCE_DIR}/3rdparty/figtree-0.9.3)
set (SCENE_PATH ${CMAKE_SOURCE_DIR}/scene)

set(DATA_BASEPATH ${CMAKE_SOURCE_DIR} CACHE STRING "")

option(CAM_ACCESS "Do you have access to the cameras(requires robot)?" OFF)
set(FLANN_BIN_PATH ${DATA_BASEPATH}/flann.bin CACHE FILEPATH "Write the path to the flann binary file" )
set(FLANN_INDEX_PATH ${DATA_BASEPATH}/flann.idx CACHE FILEPATH "Write the path to the flann index file")
set(LSH_BIN_PATH ${DATA_BASEPATH}/data.bin CACHE FILEPATH "Write the path to the lsh binary file" )
set(LSH_INDEX_PATH ${DATA_BASEPATH}/data.idx CACHE FILEPATH "Write the path to the lsh index file")
set(DB_PATH ${DATA_BASEPATH}/hands.db CACHE FILEPATH "Write the path to the SQL DB path")

configure_file(${CMAKE_SOURCE_DIR}/handclass_config.h.in ${CMAKE_SOURCE_DIR}/include/handclass_config.h)
#configure_file(${CMAKE_SOURCE_DIR}/handclass_config.h.in ${CMAKE_BINARY_DIR}/handclass_config.h)

include (FindPkgConfig)
find_package(Boost REQUIRED)
# buola is right now required for connection to the cameras and for the rendering part (scene)
find_package(Buola REQUIRED)
find_package(GSL REQUIRED)
pkg_check_modules ( OPENCV REQUIRED opencv )
include_directories (${CMAKE_SOURCE_DIR}/include
    /usr/local/include/eigen3
    ${OPENCV_INCLUDE_DIRS} ${SOURCES_DIR_CVBLOBS}
    ${INCLUDE_BUOLA} ${CMAKE_BINARY_DIR}
    ${SOURCES_DIR_UDPSOCKET}
    ${SOURCES_DIR_LSHKIT}/include
    ${SOURCES_DIR_FIGTREE}/include)

link_directories(${SOURCES_DIR_FIGTREE}/lib)

add_subdirectory(${SOURCES_DIR_FIGTREE})
add_subdirectory(${SOURCES_DIR_LSHKIT})
add_subdirectory(src)
add_subdirectory(tests)
add_subdirectory(apps)
add_subdirectory(scene)
