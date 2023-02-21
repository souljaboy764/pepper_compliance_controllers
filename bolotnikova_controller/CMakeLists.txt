find_package(xgboost REQUIRED)

add_subdirectory(src)

set(AROBASE "@")
configure_file(etc/BolotnikovaController.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/BolotnikovaController.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/BolotnikovaController.yaml" DESTINATION "${MC_CONTROLLER_INSTALL_PREFIX}/../../etc")
