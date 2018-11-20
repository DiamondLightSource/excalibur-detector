
set(CMAKE_INCLUDE_CURRENT_DIR on)
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)

include_directories(${FRAMESIMULATOR_DIR}/include ${ODINDATA_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIRS}/.. ${ZEROMQ_INCLUDE_DIRS})

file(GLOB LIB_SOURCES ExcaliburFrameSimulatorPlugin.cpp ExcaliburFrame.cpp)

# Add library for excalibur plugin
add_library(ExcaliburFrameSimulatorPlugin SHARED ${LIB_SOURCES})

install(TARGETS ExcaliburFrameSimulatorPlugin LIBRARY DESTINATION lib)