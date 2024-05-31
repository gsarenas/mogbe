cmake_minimum_required(VERSION 3.8)
project(mogbe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(python_cmake_module REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include cpp "include" directory
include_directories(include)

ament_python_install_package(${PROJECT_NAME})

# Install Python script
install(
  PROGRAMS scripts/mogbe_line_follower.py scripts/mogbe_new_goal.py scripts/mogbe_move.py 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config description img launch meshes models worlds 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
