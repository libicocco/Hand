add_executable(chandtracker_test chandtracker_test.cpp)
target_link_libraries(chandtracker_test boost_unit_test_framework boost_system ${OPENCV_LDFLAGS} handclass)

add_executable(hog_test hog_test.cpp) 
target_link_libraries(hog_test boost_unit_test_framework boost_system ${OPENCV_LDFLAGS} handclass)

add_executable(ann_test ann_test.cpp) 
target_link_libraries(ann_test boost_unit_test_framework boost_system ${OPENCV_LDFLAGS} ${GSL_LIBRARIES} lshkit)

add_executable(cDB_test cDB_test.cpp) 
target_link_libraries(cDB_test boost_regex boost_filesystem boost_system boost_unit_test_framework sqlite3 handclass)

add_executable(cPose_test cPose_test.cpp) 
target_link_libraries(cPose_test boost_regex boost_filesystem boost_system boost_unit_test_framework sqlite3 handclass)

add_executable(cPoselistMulti_test cPoselistMulti_test.cpp) 
target_link_libraries(cPoselistMulti_test boost_regex boost_filesystem boost_system boost_unit_test_framework sqlite3 ann_figtree_version figtree handclass)

add_executable(compare_hogdb_hogim compare_hogdb_hogim.cpp) 
target_link_libraries(compare_hogdb_hogim boost_regex boost_filesystem boost_system boost_unit_test_framework sqlite3 ann_figtree_version figtree ${OPENCV_LDFLAGS} handclass)

ADD_TEST(chandtracker_test ${EXECUTABLE_OUTPUT_PATH}/chandtracker_test Hello)
ADD_TEST(hog_test ${EXECUTABLE_OUTPUT_PATH}/hog_test Hello)
ADD_TEST(ann_test ${EXECUTABLE_OUTPUT_PATH}/ann_test Hello)
ADD_TEST(cDB_test ${EXECUTABLE_OUTPUT_PATH}/cDB_test Hello)
ADD_TEST(cPose_test ${EXECUTABLE_OUTPUT_PATH}/cPose_test Hello)
ADD_TEST(cPoselistMulti_test ${EXECUTABLE_OUTPUT_PATH}/cPoselistMulti_test Hello)
