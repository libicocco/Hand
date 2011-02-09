set(CMAKE_CXX_FLAGS " -O3 -std=gnu++0x -msse -msse2 -msse3 -Wall -Wextra -Winit-self -Wpointer-arith -Wno-sign-compare -Wcast-qual -Wno-unused-parameter -Wno-uninitialized -Wno-missing-field-initializers -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-promo ${CMAKE_CXX_FLAGS}")

add_executable(testhand testhand.cc)
target_link_libraries(testhand buolascene buolagui buolafsgui)
