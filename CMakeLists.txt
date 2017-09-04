# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug" "Release" CACHE PATH "Configuration types")
if(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Debug" CACHE PATH "Current build configuration")
endif(NOT CMAKE_BUILD_TYPE)

# Example Project configuration
PROJECT(CONTROLLER)

SET(CMAKE_CXX_FLAGS "-std=c++14 -Wall -Wextra -Wshadow -Wconversion -pthread")
SET(CMAKE_CXX_FLAGS_DEBUG "-O0 -DDEBUG -g")
SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

option( DISABLE_WARNINGS "Disable some of the non-critical warnings" ON )
if( DISABLE_WARNINGS )
  SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wno-return-type" )
endif( DISABLE_WARNINGS )

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/resources")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")

# core libraries
option(BUILD_CONTROLLER "Build CONTROLLER libraries" YES)

# demos
option(BUILD_CONTROLLER_DEMO "Build CONTROLLER demonstration programs and libraries" ON)

#additional dependencies
# Boost
# FIND_PACKAGE(Boost COMPONENTS system filesystem thread asio)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

###############################################################################
#
# SIMULATOR tinyXML2 library
#
###############################################################################

#add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)
add_library(tinyxml2 STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)


###############################################################################
#
# Dynamixel library
#
###############################################################################
add_library(dynamixel STATIC ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dynamixel.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dxl_hal.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dynamixel.cpp ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel/dxl_hal.cpp)
SET_TARGET_PROPERTIES(dynamixel PROPERTIES COMPILE_FLAGS "-w") #disable tons of warnings in the dynamixel library


###############################################################################
#
# Dynamixel SDK 3.4.3
#
###############################################################################
SET(DYNAMIXEL3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/dynamixel3/")
SET(DYNAMIXEL3_INCLUDE_DIR "${DYNAMIXEL3_DIR}/include")
SET(DYNAMIXEL3_SOURCE_DIR "${DYNAMIXEL3_DIR}/src")

INCLUDE_DIRECTORIES("${DYNAMIXEL3_INCLUDE_DIR}")

file(GLOB DYNAMIXEL3_HEADERS
  ${DYNAMIXEL3_INCLUDE_DIR}/dynamixel_sdk/*.h
  ${DYNAMIXEL3_INCLUDE_DIR}/dynamixel_sdk_linux/port_handler_linux.h
  ${DYNAMIXEL3_INCLUDE_DIR}/dynamixel_sdk.h
)
file(GLOB DYNAMIXEL3_SOURCES
  ${DYNAMIXEL3_SOURCE_DIR}/dynamixel_sdk/*.cpp
  ${DYNAMIXEL3_SOURCE_DIR}/dynamixel_sdk_linux/port_handler_linux.cpp
)
  
add_library(dynamixel3 STATIC ${DYNAMIXEL3_HEADERS} ${DYNAMIXEL3_SOURCES})
SET_TARGET_PROPERTIES(dynamixel3 PROPERTIES COMPILE_FLAGS "-w") #disable tons of warnings in the dynamixel library

###############################################################################
#
# CONTROLLER Hardware control library
#
###############################################################################

if(BUILD_CONTROLLER)
        file(GLOB BOARD_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Board/*.cpp"
        )
        file(GLOB BOARD_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Board/*.h"
        )
        file( GLOB BOARD_HELPERS_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Helpers/*.h"
        )
        file( GLOB BOARD_WRAPPERS_DYNAMIXEL3_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Wrappers/dynamixel3/*.cpp"
        )
        file( GLOB BOARD_WRAPPERS_DYNAMIXEL3_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Wrappers/dynamixel3/*.h"
        )
        file( GLOB BOARD_WRAPPERS_D2XX_SOURCES
            "${CMAKE_SOURCE_DIR}/src/Wrappers/d2xx/*.cpp"
        )
        file( GLOB BOARD_WRAPPERS_D2XX_HEADERS
            "${CMAKE_SOURCE_DIR}/include/Wrappers/d2xx/*.h"
        )
        add_library(Board STATIC ${BOARD_SOURCES} ${BOARD_HEADERS} ${BOARD_HELPERS_HEADERS} ${BOARD_WRAPPERS_DYNAMIXEL3_SOURCES} ${BOARD_WRAPPERS_DYNAMIXEL3_HEADERS} ${BOARD_WRAPPERS_D2XX_SOURCES} ${BOARD_WRAPPERS_D2XX_HEADERS})
        find_library(ftd2xx "libftd2xx.a")
        TARGET_LINK_LIBRARIES(Board dynamixel dynamixel3 tinyxml2 ftd2xx)
        INSTALL(TARGETS Board RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
        INSTALL(FILES ${BOARD_HEADERS} DESTINATION include/walkers/Board/)
endif(BUILD_CONTROLLER)

###############################################################################
#
# CONTROLLER demo optimizer
#
###############################################################################

if(BUILD_CONTROLLER_DEMO)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
        SET(DEMO_SOURCES ./main.cpp)
        ADD_EXECUTABLE(controller ${DEMO_SOURCES})
        TARGET_LINK_LIBRARIES(controller Board dynamixel tinyxml2)
        INSTALL(TARGETS controller RUNTIME DESTINATION bin)

endif(BUILD_CONTROLLER_DEMO)
