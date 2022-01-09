cmake_minimum_required(VERSION 3.5.1)
project(ralgo)
set( CMAKE_VERBOSE_MAKEFILE on )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CC_STANDARD 11)

set (PLATFORM_SOURCES )

if (WIN32 OR WIN64)
	set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_CRT_SECURE_NO_WARNINGS")
endif()

file(GLOB HEADERS 
	ralgo/*.h
	ralgo/**/*.h	
)

file(GLOB RALGO_SOURCES 
	ralgo/heimer/*.cpp 
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
	ralgo/signal/*.cpp
)

set(RALGO_SOURCES ${RALGO_SOURCES} PARENT_SCOPE)

file(GLOB SOURCES_TEST 
	tests/*.cpp
)

add_executable(ralgo_test ${SOURCES_TEST} ${RALGO_SOURCES})
add_library(ralgo SHARED ${RALGO_SOURCES})

target_include_directories(ralgo_test PRIVATE . tests)
target_include_directories(ralgo PRIVATE .)
target_link_libraries(ralgo_test nos igris)

if (UNIX AND NOT APPLE)
	install(TARGETS ${PROJECT_NAME}
		LIBRARY DESTINATION /usr/lib
	)

	install(DIRECTORY ${PROJECT_NAME}
    	    DESTINATION /usr/local/include
    	    PATTERN "*HIDE*" EXCLUDE
   	)
endif()


if (APPLE)
	install(TARGETS ${PROJECT_NAME}
		LIBRARY DESTINATION /usr/local/lib
	)

	install(DIRECTORY ${PROJECT_NAME}
    	    DESTINATION /usr/local/include
    	    PATTERN "*HIDE*" EXCLUDE
   	)
endif()

enable_testing()
add_test(NAME ralgo_test COMMAND ralgo_test)
