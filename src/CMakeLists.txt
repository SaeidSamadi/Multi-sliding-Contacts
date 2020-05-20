set(controller_SRC
  WipingController.cpp
  CoMQP.cpp
  FrictionEstimator.cpp
)

set(controller_HDR
  WipingController.h
  CoMQP.h
  FrictionEstimator.h
  SpaceVecAlgFilterSG.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_control eigen-quadprog::eigen-quadprog)
install(TARGETS ${PROJECT_NAME} DESTINATION "${MC_RTC_LIBDIR}")

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)
