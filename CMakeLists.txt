cmake_minimum_required(VERSION 3.14)
project(eadrc_longitudinal_controller)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/eadrc_longitudinal_controller/eadrc_longitudinal_controller.hpp
  include/eadrc_longitudinal_controller/visibility_control.hpp
  include/eadrc_longitudinal_controller/ESO.hpp
  include/eadrc_longitudinal_controller/debug_values.hpp
  include/eadrc_longitudinal_controller/smooth_stop.hpp
  include/eadrc_longitudinal_controller/lowpass_filter.hpp
  src/eadrc_longitudinal_controller.cpp
  src/ESO.cpp
  src/longitudinal_controller_utils.cpp
  src/smooth_stop.cpp
  )

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  include/eadrc_longitudinal_controller/eadrc_longitudinal_controller_node.hpp
  src/eadrc_longitudinal_controller_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "eadrc_longitudinal_controller::EadrcLongitudinalControllerNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_eadrc_longitudinal_controller
    test/test_eadrc_longitudinal_controller.cpp
  )
  ament_target_dependencies(test_eadrc_longitudinal_controller ${TEST_SOURCES})
  target_link_libraries(test_eadrc_longitudinal_controller ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
