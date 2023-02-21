add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/RandomGoalController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/RandomGoalController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/RandomGoalController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/../../etc")
