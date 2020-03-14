include_directories(..)
include_directories(.)

set(PROJECT_TEST_NAME "lidar_test")
include_directories(${GTEST_INCLUDE_DIRS} ${COMMON_INCLUDES})

add_executable(${PROJECT_TEST_NAME} lidar_test.cpp lidar_test.h)
target_link_libraries(${PROJECT_TEST_NAME} ydlidar_sdk ${GTEST_BOTH_LIBRARIES} pthread)

add_test(${PROJECT_TEST_NAME} ${PROJECT_TEST_NAME})
