# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# Example Project configuration
PROJECT(CONTROLLER)
SET(CMAKE_CXX_FLAGS "-Wall -Wextra -Wconversion -std=c++11 -pthread")
#add_definitions(-DDEBUG)

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

if(MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()

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


#Dynamixel 2
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/dynamixel2/include")
add_library(group_bulk_read SHARED 3rdParty/dynamixel2/src/*.cpp)
add_library(group_bulk_read SHARED 3rdParty/dynamixel2/src/dynamixel_sdk/*.cpp)
add_library(group_bulk_read SHARED 3rdParty/dynamixel2/src/dynamixel_sdk_linux/*.cpp)
#add_library(group_bulk_read SHARED src/dynamixel_sdk/group_bulk_read.cpp)
#add_library(group_bulk_write SHARED src/dynamixel_sdk/group_bulk_write.cpp)
#add_library(group_sync_read SHARED src/dynamixel_sdk/group_sync_read.cpp)
#add_library(group_sync_write SHARED src/dynamixel_sdk/group_sync_write.cpp)
#add_library(packet_handler SHARED src/dynamixel_sdk/packet_handler.cpp)
#add_library(port_handler SHARED src/dynamixel_sdk/port_handler.cpp)
#add_library(protocol1_packet_handler SHARED src/dynamixel_sdk/protocol1_packet_handler.cpp)
#add_library(protocol2_packet_handler SHARED src/dynamixel_sdk/protocol2_packet_handler.cpp)
#add_library(port_handler_linux SHARED src/dynamixel_sdk_linux/port_handler_linux.cpp)
add_executable(Projekt_Dynamixel src/read_write.cpp)
target_link_libraries(Projekt_Dynamixel group_bulk_read group_bulk_write group_sync_read
group_sync_write packet_handler port_handler protocol1_packet_handler protocol2_packet_handler port_handler_linux)


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
        add_library(Board STATIC ${BOARD_SOURCES} ${BOARD_HEADERS})
        TARGET_LINK_LIBRARIES(Board dynamixel tinyxml2)
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
