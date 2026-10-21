cmake_minimum_required(VERSION 3.8)
project(nav2_ec_dwvp_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)

nav2_package()
set(CMAKE_CXX_STANDARD 17)

include_directories(include)

set(dependencies
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_regulated_pure_pursuit_controller
  nav2_util
  nav_msgs
  pluginlib
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
)

set(library_name nav2_ec_dwvp_controller)

add_library(${library_name} SHARED
  src/ec_dwvp_controller.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

pluginlib_export_plugin_description_file(
  nav2_core
  nav2_ec_dwvp_controller.xml
)

install(DIRECTORY config docs DESTINATION share/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_executable(solver_fixture_runner test/solver_fixture_runner.cpp)
  target_link_libraries(solver_fixture_runner ${library_name})
  ament_target_dependencies(solver_fixture_runner ${dependencies})
  ament_add_test(solver_parity
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_solver_fixtures.py
      $<TARGET_FILE:solver_fixture_runner> ${CMAKE_CURRENT_SOURCE_DIR}/test/solver_cases.json
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  add_executable(vp_scaled_fixture_runner test/vp_scaled_fixture_runner.cpp)
  target_link_libraries(vp_scaled_fixture_runner ${library_name})
  ament_target_dependencies(vp_scaled_fixture_runner ${dependencies})
  ament_add_test(vp_scaled_parity
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_solver_fixtures.py
      $<TARGET_FILE:vp_scaled_fixture_runner> ${CMAKE_CURRENT_SOURCE_DIR}/test/vp_scaled_cases.json
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  # Simulator b09ba8e3a2a5feced5ba317adf793b0e33e69591; fixture copied unchanged.
  add_executable(desired_vector_fixture_runner test/desired_vector_fixture_runner.cpp)
  target_link_libraries(desired_vector_fixture_runner ${library_name})
  ament_target_dependencies(desired_vector_fixture_runner ${dependencies})
  ament_add_test(desired_vector_parity
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/check_desired_vector_fixtures.py
      $<TARGET_FILE:desired_vector_fixture_runner>
      ${CMAKE_CURRENT_SOURCE_DIR}/test/desired_vector_cases.json
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  )
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_link_libraries(test_controller ${library_name})
  ament_target_dependencies(test_controller ${dependencies})
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
