set(FRAMERECEIVER_DIR ${DATA_DIR}/frameReceiver)
set(FRAMEPROCESSOR_DIR ${DATA_DIR}/frameProcessor)
set(FRAMESIMULATOR_DIR ${DATA_DIR}/frameSimulator)
set(TEST_DIR ${DATA_DIR}/test)

# Add common/include directory to include path
include_directories(${DATA_DIR}/common/include)

add_subdirectory(${FRAMERECEIVER_DIR})
add_subdirectory(${FRAMEPROCESSOR_DIR})
add_subdirectory(${FRAMESIMULATOR_DIR})
add_subdirectory(${TEST_DIR})
#add_subdirectory(doc)
add_subdirectory(config)
