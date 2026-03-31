cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

# Installiere URDF/Xacro Dateien
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Installiere Launch Files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
