add_executable(chandtracker_gui chandtracker_gui.cpp)
target_link_libraries(chandtracker_gui ${OPENCV_LDFLAGS} boost_system handclass)

add_executable(hog_gui hog_gui.cpp) 
target_link_libraries(hog_gui boost_unit_test_framework boost_system ${OPENCV_LDFLAGS} handclass)

#add_executable(nn_hogJointsParts nn_hogJointsParts.cpp) 
#target_link_libraries(nn_hogJointsParts boost_system boost_filesystem boost_regex handclass sqlite3 ${GSL_LIBRARIES} lshkit ${OPENCV_LDFLAGS})

add_executable(handTracker handTracker.cpp) 
target_link_libraries(handTracker boost_regex boost_filesystem boost_system boost_unit_test_framework boost_program_options sqlite3 ann_figtree_version figtree handclass ${OPENCV_LDFLAGS})

add_executable(createFeatDB createFeatDB.cpp) 
target_link_libraries(createFeatDB boost_system boost_filesystem boost_regex handclass sqlite3 ${GSL_LIBRARIES} lshkit ${OPENCV_LDFLAGS})
