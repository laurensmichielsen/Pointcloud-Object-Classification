cmake_minimum_required(VERSION 3.8)
project(rviz_interface_plugin)

set(CMAKE_BUILD_TYPE Debug)

if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif ()
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(labeling_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(jsoncpp QUIET)
find_package(PCL REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)

set(CMAKE_AUTOMOC ON)

include_directories(include ${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${PROJECT_NAME} SHARED
        include/label_panel.hpp
        src/label_panel.cpp)

ament_target_dependencies(${PROJECT_NAME}
        rclcpp rviz_common rviz_default_plugins
        interactive_markers
        custom_msgs
        labeling_msgs
        pcl_conversions
        )
# TODO: change
target_link_libraries(${PROJECT_NAME}
        jsoncpp_lib
        ${PCL_LIBARIES}
)

target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

ament_package()
