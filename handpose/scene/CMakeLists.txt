add_executable(testhand testhand.cc)
target_link_libraries(testhand buolascene buolagui buolafsgui boost_system)

add_executable(renderhand renderhand.cc)
target_link_libraries(renderhand buolascene buolagui buolafsgui buolacv boost_system boost_filesystem
                      opencv_highgui sqlite3 handclass)
