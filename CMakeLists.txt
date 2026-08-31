cmake_minimum_required(VERSION 3.8)
project(cs685_fall26_materials)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteCartesianPoint.action"
  "action/ExecuteJointPose.action"
  DEPENDENCIES geometry_msgs
)

# Install course resources as they are added to the package.
foreach(resource_dir launch config rviz maps worlds casas)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_dir}")
    install(
      DIRECTORY "${resource_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

install(
  FILES GettingStartedMac.md
  DESTINATION "share/${PROJECT_NAME}"
)

# Install milestone II as an importable Python package. The generated console
# scripts use the interpreter selected at build time, while --symlink-install
# keeps edits to the Python modules live without another build.
ament_python_install_package(
  cs685_milestoneII
  PACKAGE_DIR scripts/milestoneII
  SETUP_CFG milestoneII_setup.cfg
  SCRIPTS_DESTINATION "lib/${PROJECT_NAME}"
)

install(
  PROGRAMS
    scripts/autonomous_explorer.py
    scripts/controller.py
    scripts/detect_objects_trusted.py
    scripts/kitchen_item_traveler.py
    scripts/large_house_mujoco_driver.py
    scripts/locomotion_posture_guard.py
    scripts/minimal_mujoco_driver.py
    scripts/topdown_mujoco_driver.py
    scripts/topdown_video_recorder.py
  DESTINATION "lib/${PROJECT_NAME}"
)

ament_package()
