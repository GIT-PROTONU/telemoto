cmake_minimum_required(VERSION 3.22)
project(telamoto_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/ur_rtde_joint_pub.py
  scripts/ur_dashboard_autoplay.py
  scripts/ur_servo_controller.py
  scripts/scene_wall.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
