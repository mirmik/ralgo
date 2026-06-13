cmake_minimum_required(VERSION 3.5.1)
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
	
if(RALGO_TOP_LEVEL)
	set(CMAKE_CXX_STANDARD 20)
	set(CMAKE_C_STANDARD 11)
	set( CMAKE_VERBOSE_MAKEFILE on )
	
	if (WIN32 OR WIN64)
		set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS")
	endif()
	
	add_library(ralgo SHARED ${RALGO_SOURCES})
	
	target_include_directories(ralgo
		PUBLIC
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
			$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
	)
	target_link_libraries(ralgo PUBLIC nos igris)

	if(RALGO_BUILD_TESTS)
		add_executable(ralgo_test ${SOURCES_TEST})
		target_include_directories(ralgo_test PRIVATE . tests)
		target_link_libraries(ralgo_test PRIVATE ralgo)
	endif()
	
	if (UNIX AND NOT APPLE)
		install(TARGETS ${PROJECT_NAME}
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
			ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
			RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
	
		install(DIRECTORY ${PROJECT_NAME}
	    	    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	    	    PATTERN "*HIDE*" EXCLUDE
	   	)
	endif()
	
	
	if (APPLE)
		install(TARGETS ${PROJECT_NAME}
			LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
			ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
			RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
	
		install(DIRECTORY ${PROJECT_NAME}
	    	    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
	    	    PATTERN "*HIDE*" EXCLUDE
	   	)
	endif()
	
	if(RALGO_BUILD_TESTS)
		enable_testing()
		add_test(NAME ralgo_test COMMAND ralgo_test)
	endif()
	
else()
	set(RALGO_SOURCES ${RALGO_SOURCES} PARENT_SCOPE)
endif()
