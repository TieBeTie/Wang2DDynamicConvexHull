find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include)

add_executable(pennant_tests
    test_main.cpp
)

target_link_libraries(pennant_tests
    PRIVATE
        GTest::gtest_main
)

add_test(NAME PennantTests COMMAND pennant_tests)

add_custom_command(
    TARGET pennant_tests POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
            ${PROJECT_SOURCE_DIR}/data
            ${CMAKE_CURRENT_BINARY_DIR}/data
)
