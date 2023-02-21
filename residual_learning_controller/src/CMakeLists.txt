include_directories(${roscpp_INCLUDE_DIRS} ${check_selfcollision_INCLUDE_DIRS})
link_libraries(${roscpp_LIBRARIES} ${check_selfcollision_LIBRARIES})

set(SRC
  ResidualLearningController.cpp
)

set(HDR
  ResidualLearningController.h
  Model.hpp
)

add_controller(ResidualLearningController ${SRC} ${HDR})
set_target_properties(ResidualLearningController PROPERTIES COMPILE_FLAGS "-DResidualLearningController_EXPORTS")
set_property(TARGET ResidualLearningController PROPERTY CXX_STANDARD 14)