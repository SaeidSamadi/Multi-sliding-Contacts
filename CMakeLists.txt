cmake_minimum_required(VERSION 3.1.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME WipingController)
set(PROJECT_DESCRIPTION "WipingController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)
find_package(eigen-quadprog REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/WipingController.in.conf "${CMAKE_CURRENT_BINARY_DIR}/etc/WipingController.conf")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/WipingController.conf" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

