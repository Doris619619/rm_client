cmake_minimum_required(VERSION 3.8)
project(rm_client)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)

# Paho MQTT C++
find_library(PAHO_MQTTPP_LIB paho-mqttpp3)
if(NOT PAHO_MQTTPP_LIB)
	message(FATAL_ERROR "Could not find Paho MQTT C++ library: paho-mqttpp3")
endif()

# Paho MQTT C (prefer async SSL variant, fallback to async non-SSL)
find_library(PAHO_MQTT_C_LIB paho-mqtt3as)
if(NOT PAHO_MQTT_C_LIB)
	find_library(PAHO_MQTT_C_LIB paho-mqtt3a)
endif()
if(NOT PAHO_MQTT_C_LIB)
	message(FATAL_ERROR "Could not find Paho MQTT C library: paho-mqtt3as or paho-mqtt3a")
endif()

set(PROTO_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/proto/guard_ctrl.proto
	${CMAKE_CURRENT_SOURCE_DIR}/proto/judge_system.proto
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

set(COMMON_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_BINARY_DIR}
	${Protobuf_INCLUDE_DIRS}
)

add_executable(semi_auto_keyboard
	src/semi_auto_keyboard.cpp
)

target_include_directories(semi_auto_keyboard PRIVATE ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(semi_auto_keyboard rclcpp std_msgs)

add_executable(judge_sender
	src/judge_sender.cpp
	${PROTO_SRCS}
	${PROTO_HDRS}
)

target_include_directories(judge_sender PRIVATE ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(judge_sender rclcpp std_msgs)
target_link_libraries(judge_sender
	${Protobuf_LIBRARIES}
	${PAHO_MQTTPP_LIB}
	${PAHO_MQTT_C_LIB}
)

add_executable(judge_receiver
	src/judge_receiver.cpp
	src/state_store.cpp
	src/topic_registry.cpp
	${PROTO_SRCS}
	${PROTO_HDRS}
)

target_include_directories(judge_receiver PRIVATE ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(judge_receiver rclcpp std_msgs geometry_msgs)
target_link_libraries(judge_receiver
	${Protobuf_LIBRARIES}
	${PAHO_MQTTPP_LIB}
	${PAHO_MQTT_C_LIB}
)

add_executable(mock_custom_image_pub
	src/mock_custom_image_pub.cpp
	${PROTO_SRCS}
	${PROTO_HDRS}
)

target_include_directories(mock_custom_image_pub PRIVATE ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(mock_custom_image_pub rclcpp)
target_link_libraries(mock_custom_image_pub
	${Protobuf_LIBRARIES}
	${PAHO_MQTTPP_LIB}
	${PAHO_MQTT_C_LIB}
)

add_executable(camera_custom_image_pub
	src/camera_custom_image_pub.cpp
)

target_include_directories(camera_custom_image_pub PRIVATE ${COMMON_INCLUDE_DIRS})
ament_target_dependencies(camera_custom_image_pub rclcpp std_msgs)
target_link_libraries(camera_custom_image_pub
	${OpenCV_LIBRARIES}
)

install(TARGETS
	semi_auto_keyboard
	judge_sender
	judge_receiver
	mock_custom_image_pub
	camera_custom_image_pub
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	set(ament_cmake_copyright_FOUND TRUE)
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
