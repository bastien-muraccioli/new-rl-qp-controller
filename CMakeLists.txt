cmake_minimum_required(VERSION 3.22)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME H1RLQPController)
set(PROJECT_DESCRIPTION "H1RLQPController")
set(PROJECT_URL "")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)
include(CTest)
enable_testing()

if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

add_subdirectory(src)

# if(BUILD_TESTING)
#   add_subdirectory(tests)
# endif()

configure_file(etc/H1RLQPController.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/etc/H1RLQPController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/H1RLQPController.yaml"
        DESTINATION "${MC_CONTROLLER_RUNTIME_INSTALL_PREFIX}/etc")
