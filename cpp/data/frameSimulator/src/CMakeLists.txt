
set(CMAKE_INCLUDE_CURRENT_DIR on)
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK)

include_directories(${FRAMESIMULATOR_DIR}/include ${ODINDATA_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIRS}/.. ${ZEROMQ_INCLUDE_DIRS} ${PCAP_INCLUDE_DIR})

file(GLOB LIB_SOURCES ExcaliburFrameSimulatorPlugin.cpp)

# Add library for excalibur plugin
add_library(ExcaliburFrameSimulatorPlugin SHARED ${LIB_SOURCES})

target_link_libraries(ExcaliburFrameSimulatorPlugin ${PCAP_LIBRARY})

install(TARGETS ExcaliburFrameSimulatorPlugin LIBRARY DESTINATION lib)
