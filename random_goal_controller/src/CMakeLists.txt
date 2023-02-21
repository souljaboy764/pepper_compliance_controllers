include_directories(${roscpp_INCLUDE_DIRS} ${check_selfcollision_INCLUDE_DIRS})
link_libraries(${roscpp_LIBRARIES} ${check_selfcollision_LIBRARIES})

set(SRC
  RandomGoalController.cpp
)

set(HDR
  RandomGoalController.h
)

add_controller(RandomGoalController ${SRC} ${HDR})
set_target_properties(RandomGoalController PROPERTIES COMPILE_FLAGS "-DRandomGoalController_EXPORTS")