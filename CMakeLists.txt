cmake_minimum_required(VERSION 3.1.1)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)

set(PROJECT_NAME WipingController)
set(PROJECT_DESCRIPTION "WipingController")
set(PROJECT_URL "")

project(${PROJECT_NAME} CXX)

find_package(mc_rtc REQUIRED)
find_package(eigen-quadprog REQUIRED)

if(EXISTS "${MC_RTC_LIBDIR}/mc_controller/etc/WipingController.conf")
message(FATAL_ERROR "\n \
############################################################ \n \
   Please uninstall the previous version (make uninstall)    \n \
############################################################ \n \
")
endif()

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/WipingController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/WipingController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/WipingController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/etc")

