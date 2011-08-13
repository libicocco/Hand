add_executable(testhand testhand.cc csavebutton.cpp loadPose.cpp)
target_link_libraries(testhand buolascene buolagui buolafsgui boost_system boost_filesystem)

add_executable(renderhand renderhand.cc csavebutton.cpp loadPose.cpp handRenderer.cpp)
target_link_libraries(renderhand buolascene buolagui buolafsgui buolacv boost_system boost_filesystem
  opencv_highgui sqlite3 handclass)

add_executable(renderPoses  renderPoses.cpp csavebutton.cpp loadPose.cpp handRenderer.cpp)
target_link_libraries(renderPoses buolascene buolagui buolafsgui buolacv boost_system boost_filesystem opencv_highgui sqlite3 handclass)
