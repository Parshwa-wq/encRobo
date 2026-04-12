cmake_minimum_required(VERSION 3.8)
project(enc)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  description
  launch
  meshes
  enc
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python node
install(PROGRAMS
  enc/odometry_publisher.py
  enc/slip_detector.py
  enc/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
