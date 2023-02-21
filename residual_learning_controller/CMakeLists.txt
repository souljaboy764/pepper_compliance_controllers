add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/ResidualLearningController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/ResidualLearningController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/ResidualLearningController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/../../etc")
