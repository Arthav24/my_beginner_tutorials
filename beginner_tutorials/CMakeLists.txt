cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# clangd support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
execute_process(COMMAND "rm" "-f" "${CMAKE_SOURCE_DIR}/compile_commands.json")
execute_process(COMMAND "ln" "-s" "-r" "${CMAKE_BINARY_DIR}/compile_commands.json" "${CMAKE_SOURCE_DIR}/")

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(beginner_tutorial_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(catch_ros2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(talker src/publisher_member_function.cc)
ament_target_dependencies(talker rclcpp std_msgs beginner_tutorial_interfaces tf2 tf2_ros)


add_executable(listener src/subscriber_member_function.cc)
ament_target_dependencies(listener rclcpp std_msgs beginner_tutorial_interfaces)

install(TARGETS
        talker
        listener
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)


if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_lint_cmake_FOUND TRUE)
    set(ament_cmake_flake8_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    # build the integration test
    ###########################################################################
    # 1.) BUILD AND INSTALL THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
    ###########################################################################
    # This is the node in which integration tests occur
    add_executable(integration_test_node
            test/integration_test_node.cc
    )
    # The link libraries call this node with "catch_ros2::catch_ros2_with_node_main"
    # to get the default integration test node main function
    target_link_libraries(integration_test_node
            catch_ros2::catch_ros2_with_node_main
    )
    ament_target_dependencies(integration_test_node
            rclcpp std_srvs std_msgs beginner_tutorial_interfaces
    )
    install(TARGETS
            integration_test_node
            DESTINATION lib/${PROJECT_NAME}
    )

    ######################################
    ## 2.) ADD INTEGRATION TEST SUPPORT ##
    ######################################
    # This CMake function allows the integration test to be run
    # when running "colcon test".
    catch_ros2_add_integration_test (IntegrationTest_beginnerTutorials
            LAUNCH_FILE integration_test.launch.yaml  # example in YAML
    )
endif ()

ament_package()
