cmake_minimum_required(VERSION 3.8)
project(amnis_controller)

# Enforce reasonable C/C++ standards even though this is a Python package
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JoystickCommand.msg"
  "msg/PowertrainCommand.msg"
  "msg/SteerCommand.msg"
  "msg/BrakeCommand.msg"
  "msg/SensorData.msg"
  DEPENDENCIES sensor_msgs
)

# Get the proper Python installation directory
if(NOT DEFINED PYTHON_INSTALL_DIR)
  set(PYTHON_INSTALL_DIR "local/lib/python3.12/dist-packages")
endif()

# Install Python modules manually to avoid target name conflicts
install(
  DIRECTORY amnis_controller/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
)

# Install console scripts
install(PROGRAMS
  amnis_controller/joystick_normalizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joystick_normalizer_node
)

install(PROGRAMS
  amnis_controller/vehicle_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vehicle_controller_node
)

install(PROGRAMS
  amnis_controller/steer_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME steer_controller_node
)

install(PROGRAMS
  amnis_controller/topic_aggregator_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME topic_aggregator_node
)

install(PROGRAMS
  amnis_controller/brake_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME brake_controller_node
)

install(PROGRAMS
  amnis_controller/powertrain_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME powertrain_controller_node
)

install(PROGRAMS
  amnis_controller/sensor_reader_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_reader_node
)

install(
  DIRECTORY launch resource
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY test
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
