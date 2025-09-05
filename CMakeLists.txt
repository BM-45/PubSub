cmake_minimum_required(VERSION 3.8)
project(node1)

set(CMAKE_CXX_STANDARD 17)

set(node1_MAJOR_VERSION 0)
set(node1_MINOR_VERSION 0)
set(node1_PATCH_VERSION 0)
set(node1_VERSION ${node1_MAJOR_VERSION}.${node1_MINOR_VERSION}.${node1_PATCH_VERSION})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generating custom message
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/BasicMessage.msg"
	DEPENDENCIES builtin_interfaces
)

add_executable(talker src/basic_publisher.cpp)
ament_target_dependencies(talker rclcpp )
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(talker "${cpp_typesupport_target}") 

add_executable(listener src/basic_subscriber.cpp)
ament_target_dependencies(listener rclcpp )
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(listener "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)


# Install
install(TARGETS
talker
listener
DESTINATION lib/${PROJECT_NAME})

ament_package()
