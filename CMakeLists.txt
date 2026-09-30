cmake_minimum_required(VERSION 3.18)
project(ralgo)

include(GNUInstallDirs)

set(RALGO_TOP_LEVEL OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
	set(RALGO_TOP_LEVEL ON)
endif()

option(RALGO_BUILD_TESTS "Build ralgo tests" ${RALGO_TOP_LEVEL})

file(GLOB HEADERS 
	ralgo/*.h
	ralgo/**/*.h	
)

file(GLOB RALGO_SOURCES 
	ralgo/heimer/*.cpp 
	ralgo/oldheimer/*.cpp  
	ralgo/*.cpp
	ralgo/trajectory/*.c
	ralgo/trajectory/*.cpp
	ralgo/log/log.cpp
	ralgo/log/log-console.cpp
	ralgo/linalg/*.cpp
	ralgo/clinalg/*.cpp
	ralgo/lp/*.c
	ralgo/lp/*.cpp
	ralgo/util/*.c
	ralgo/robo/*.cpp
	ralgo/imu/*.cpp
	ralgo/rxsignal/*.cpp
	ralgo/signal/*.cpp
	ralgo/cnc/*.cpp
)

file(GLOB SOURCES_TEST CONFIGURE_DEPENDS
	tests/*.cpp
	tests/cnc/*.cpp
)
	
foreach(dep igris nos)
    if(NOT TARGET ${dep} AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../${dep}/CMakeLists.txt")
        add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../${dep}" "${CMAKE_CURRENT_BINARY_DIR}/${dep}")
    endif()
endforeach()
if(NOT TARGET rxcpp)
    find_path(RXCPP_INCLUDE_DIR rxcpp/rx.hpp HINTS "${CMAKE_CURRENT_SOURCE_DIR}/../RxCpp/Rx/v2/src" REQUIRED)
    add_library(rxcpp INTERFACE)
    target_include_directories(rxcpp INTERFACE "${RXCPP_INCLUDE_DIR}")
endif()
add_library(ralgo SHARED ${RALGO_SOURCES})
target_compile_features(ralgo PUBLIC cxx_std_20)
target_include_directories(ralgo PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(ralgo PUBLIC nos igris rxcpp)
set_target_properties(ralgo PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
if(RALGO_BUILD_TESTS)
    enable_testing()
    add_executable(ralgo_test ${SOURCES_TEST})
    target_include_directories(ralgo_test PRIVATE . tests)
    target_link_libraries(ralgo_test PRIVATE ralgo)
    add_test(NAME ralgo_test COMMAND ralgo_test)
endif()
install(TARGETS ralgo LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ralgo DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} PATTERN "*HIDE*" EXCLUDE)
if(NOT RALGO_TOP_LEVEL)
    set(RALGO_SOURCES ${RALGO_SOURCES} PARENT_SCOPE)
endif()
