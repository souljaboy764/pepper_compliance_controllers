include_directories(${roscpp_INCLUDE_DIRS} ${check_selfcollision_INCLUDE_DIRS})
link_libraries(${roscpp_LIBRARIES} ${check_selfcollision_LIBRARIES})

set(SRC
  GravityController.cpp
)

set(HDR
  GravityController.h
)

add_controller(GravityController ${SRC} ${HDR})
set_target_properties(GravityController PROPERTIES COMPILE_FLAGS "-DGravityController_EXPORTS")