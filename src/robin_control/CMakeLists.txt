cmake_minimum_required(VERSION 3.8)
project(robin_control)

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/node_encoder_sim.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motor_encoder
)

install(PROGRAMS
  src/node_inverse_kinematic.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME inverse_kinematic
)

install(PROGRAMS
  src/node_odometry.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_odometry
)

install(PROGRAMS
  src/node_motion.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion
)


ament_package()
