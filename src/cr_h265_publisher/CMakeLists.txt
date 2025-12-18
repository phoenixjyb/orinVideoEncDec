cmake_minimum_required(VERSION 3.16)
project(cr_h265_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_library(gst_h265_encoder
  src/gst_h265_encoder.cpp
)
target_include_directories(gst_h265_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GST_INCLUDE_DIRS}
)
target_link_directories(gst_h265_encoder PUBLIC ${GST_LIBRARY_DIRS})
target_link_libraries(gst_h265_encoder PUBLIC ${GST_LIBRARIES})
ament_target_dependencies(gst_h265_encoder PUBLIC rclcpp)

add_executable(cr_h265_publisher_node
  src/cr_h265_publisher_node.cpp
)
target_include_directories(cr_h265_publisher_node PRIVATE
  ${GST_INCLUDE_DIRS}
)
target_link_directories(cr_h265_publisher_node PRIVATE ${GST_LIBRARY_DIRS})
target_link_libraries(cr_h265_publisher_node gst_h265_encoder ${GST_LIBRARIES})
ament_target_dependencies(cr_h265_publisher_node rclcpp sensor_msgs)

add_executable(cr_h265_dump_node
  src/cr_h265_dump_node.cpp
)
ament_target_dependencies(cr_h265_dump_node rclcpp sensor_msgs)

install(TARGETS
  gst_h265_encoder
  cr_h265_publisher_node
  cr_h265_dump_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
