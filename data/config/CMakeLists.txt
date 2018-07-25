file(GLOB CONFIG_FILES 
  *.config
  *.xml
  )
  
message(STATUS "Config files: ${CONFIG_FILES}")

set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/config/data")

foreach(CONFIG_FILE ${CONFIG_FILES})
  get_filename_component(CONFIG_FILE_NAME ${CONFIG_FILE} NAME)
  install(
    CODE 
    "
      set(CONFIG_INSTALL_DIR \"${CONFIG_INSTALL_DIR}\")
      set(LIB_INSTALL_DIR \"${CMAKE_INSTALL_PREFIX}/lib\")
      message(STATUS \"Installing ${CONFIG_INSTALL_DIR}/${CONFIG_FILE_NAME}\")
      configure_file(
        ${CONFIG_FILE} ${CONFIG_INSTALL_DIR}/${CONFIG_FILE_NAME} @ONLY
      )
    "
    )
endforeach(CONFIG_FILE)
