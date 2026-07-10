cmake_minimum_required(VERSION 3.8)
project(cs685_fall26_materials)

find_package(ament_cmake REQUIRED)

# Install course resources as they are added to the package.
foreach(resource_dir launch config rviz maps worlds)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${resource_dir}")
    install(
      DIRECTORY "${resource_dir}"
      DESTINATION "share/${PROJECT_NAME}"
    )
  endif()
endforeach()

install(
  FILES GettingStarted.md
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  PROGRAMS scripts/detect_objects_trusted.py
  DESTINATION "lib/${PROJECT_NAME}"
)

ament_package()
