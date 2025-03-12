cmake_minimum_required(VERSION 3.10)
find_package(GTest REQUIRED)
enable_testing()

add_executable(my_test my_test.cpp)

target_include_directories(my_test PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

target_link_libraries(my_test GTest::GTest GTest::Main)
gtest_discover_tests(my_test)