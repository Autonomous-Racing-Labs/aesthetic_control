cmake_minimum_required(VERSION 3.5)
project(aesthetic_control_interfaces)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


set(msg_files
  "msg/UnderglowColor.msg"
  )

set(srv_files
  "srv/BrakeLights.srv"
  "srv/HazardLights.srv"
  "srv/Headlights.srv"
  "srv/HighBeams.srv"
  "srv/ReverseLights.srv"
  "srv/Underglow.srv"
  )

rosidl_generate_interfaces(aesthetic_control_interfaces
  ${msg_files}
  ${srv_files}
  DEPENDENCIES
    builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
