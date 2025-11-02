cmake_minimum_required(VERSION 3.8)
project(ferrari)

# 혼합 빌드
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# C++ 플러그인 의존성
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)

# C++ 컨트롤러 플러그인 라이브러리
add_library(f1tenth_commands_controller SHARED
  src/commands_to_joints_controller.cpp
)

target_include_directories(f1tenth_commands_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(f1tenth_commands_controller
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
)

# pluginlib 메타 등록
pluginlib_export_plugin_description_file(controller_interface plugins.xml)

# Python 패키지 설치(기존 setup.py 사용)
ament_python_install_package(${PROJECT_NAME})

# scripts 디렉토리가 있을 때만
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  file(GLOB _PY_SCRIPTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
  if(_PY_SCRIPTS)
    install(PROGRAMS ${_PY_SCRIPTS}
      DESTINATION lib/${PROJECT_NAME})
  endif()
endif()

# 리소스 설치
install(TARGETS f1tenth_commands_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include OPTIONAL)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/  DESTINATION share/${PROJECT_NAME}/config OPTIONAL)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch OPTIONAL)
install(DIRECTORY urdf/  DESTINATION share/${PROJECT_NAME}/urdf OPTIONAL)
install(DIRECTORY maps/  DESTINATION share/${PROJECT_NAME}/maps OPTIONAL)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/ferrari/key_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME key_teleop        # 확장자 없이 설치 → ros2 run ferrari key_teleop
)
ament_package()
