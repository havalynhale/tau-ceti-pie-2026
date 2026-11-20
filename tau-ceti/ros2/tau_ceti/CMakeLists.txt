cmake_minimum_required(VERSION 3.8)
project(tau_ceti)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/teleop.py scripts/imu_tf.py scripts/gamepad_bridge.py DESTINATION lib/${PROJECT_NAME})

ament_package()
