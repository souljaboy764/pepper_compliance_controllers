include_directories(${roscpp_INCLUDE_DIRS} ${check_selfcollision_INCLUDE_DIRS})
link_libraries(${roscpp_LIBRARIES} ${check_selfcollision_LIBRARIES} xgboost::xgboost)

set(SRC
  BolotnikovaController.cpp
)

set(HDR
  BolotnikovaController.h
)

add_controller(BolotnikovaController ${SRC} ${HDR})
set_target_properties(BolotnikovaController PROPERTIES COMPILE_FLAGS "-DBolotnikovaController_EXPORTS")