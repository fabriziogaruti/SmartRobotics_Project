cmake_minimum_required(VERSION 3.5)
project(robot)
# enable_language(py)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rclpy REQUIRED)
# find_package(PythonInterp)
# find_package(PythonLibs)
# INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
# INCLUDE_DIRECTORIES("/usr/local/include")
# LINK_DIRECTORIES(/usr/local/lib)
# OPTION(BUILD_SHARED_LIBS "turn OFF for .a libs" ON)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# add_executable(move_arm scripts/move_arm.py)
# ament_target_dependencies(move_arm rclpy)
# set_target_properties(move_arm PROPERTIES LINKER_LANGUAGE py)


install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# install(
#   PROGRAMS scripts/move_arm.py
#   PROGRAMS scripts/move_arm
#  DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS
#   move_arm
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_package()
