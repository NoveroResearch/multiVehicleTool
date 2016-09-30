cmake_minimum_required(VERSION 2.6)
project(multiVehicleTool)
set(multiVehicleTool_VERSION_MAJOR 0)
set(multiVehicleTool_VERSION_MINOR 1)
include(ExternalProject)

# --- Cancel if not build on Linux ---

IF(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
	MESSAGE(WARNING "This will only compile on Linux because it depends on the Linux Bluetooth Stack.")
ENDIF(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Linux")	

# --- Check Mandatory Build-time Dependencies ---

set(CMAKE_MODULE_PATH ${multiVehicleTool_SOURCE_DIR}/cmake/modules ${CMAKE_MODULE_PATH})

find_package(Boost REQUIRED)
find_package(Boost COMPONENTS system filesystem thread program_options REQUIRED)
include_directories(system ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})
link_libraries(${Boost_LIBRARIES})

find_package(PkgConfig)
pkg_search_module(GLIB2 REQUIRED glib-2.0)
include(FindReadline)
include(FindBluez)

# --- Include Anki Drive SDK ---

ExternalProject_Add(drivesdk
	URL https://github.com/anki/drive-sdk/archive/v0.3.0.tar.gz
	PREFIX ${PROJECT_SOURCE_DIR}/dep/drivesdk
	INSTALL_COMMAND ""
	PATCH_COMMAND wget -q -O - https://github.com/anki/drive-sdk/compare/v0.3.0...bd41b09.diff | patch -p1 && echo patched to commit bd41b09 COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/cmake/fix-light-time.patch && echo Applied fix-light-time.patch COMMAND patch -p1 < ${PROJECT_SOURCE_DIR}/cmake/fix-cmake-test.patch && echo Applied fix-cmake-test.patch
)



ExternalProject_Get_Property(drivesdk SOURCE_DIR)
set(DRIVESDK_INCLUDE_DIR ${SOURCE_DIR}/include)

ExternalProject_Get_Property(drivesdk BINARY_DIR)
set(DRIVESDK_LIBRARY_PATH ${BINARY_DIR}/src/${CMAKE_FIND_LIBRARY_PREFIXES}ankidrive.a)
set(DRIVESDK_LIBRARY ankidrive)
add_library(${DRIVESDK_LIBRARY} UNKNOWN IMPORTED)
set_property(TARGET ${DRIVESDK_LIBRARY} PROPERTY IMPORTED_LOCATION ${DRIVESDK_LIBRARY_PATH})
add_dependencies(${DRIVESDK_LIBRARY} drivesdk)

# --- Set Compiler Switches ---

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -Wall -Werror -g -Wmissing-braces")
endif()

# --- Create Global Configuration Header File ---

configure_file("${PROJECT_SOURCE_DIR}/cmake/config.h.in" "${PROJECT_BINARY_DIR}/inc/multiVehicleTool/config.h")

# --- Collect Sources and Headers ---

set(multiVehicleTool_SOURCES
	${PROJECT_SOURCE_DIR}/src/multiVehicleTool.cpp
	${PROJECT_SOURCE_DIR}/src/CommandPrompt.cpp
	${PROJECT_SOURCE_DIR}/src/Vehicle.cpp
	${PROJECT_SOURCE_DIR}/src/VehicleManager.cpp
	${PROJECT_SOURCE_DIR}/src/VehicleModel.cpp
	${PROJECT_SOURCE_DIR}/src/VehicleScanner.cpp
	${PROJECT_SOURCE_DIR}/src/VehicleModel.cpp
	${PROJECT_SOURCE_DIR}/src/util/AnkiDriveProtocol.cpp
	${PROJECT_SOURCE_DIR}/src/util/PidFile.cpp
)

file(GLOB_RECURSE multiVehicleTool_HEADERS ${PROJECT_SOURCE_DIR}/inc/multiVehicleTool/*.h ${PROJECT_BINARY_DIR}/inc/multiVehicleTool/*.h)

# --- Set Include Directories ---

include_directories(
	"${PROJECT_BINARY_DIR}/inc"
	"${PROJECT_SOURCE_DIR}/inc"
)

# --- Create Targets ---

add_executable(multiVehicleTool ${multiVehicleTool_SOURCES})

#make sure bluez is built before building multiVehicleTool
add_dependencies(multiVehicleTool bluez)

if(NOT NOSETCAP)
	add_custom_command(TARGET multiVehicleTool POST_BUILD COMMAND /usr/bin/sudo /sbin/setcap 'cap_net_raw,cap_net_admin+eip' $<TARGET_FILE:multiVehicleTool> || true)
endif()

include_directories(system
	${BLUEZ_INCLUDE_DIRS}
	${GLIB2_INCLUDE_DIRS}
	${DBUS_INCLUDE_DIRS}
	${READLINE_INCLUDE_DIR}
	${DRIVESDK_INCLUDE_DIR}
)

target_link_libraries(multiVehicleTool
	${BLUEZ_LIBRARIES}
	${GLIB2_LIBRARIES}
	${DBUS_LIBRARIES}
	${READLINE_LIBRARY}
	${DRIVESDK_LIBRARY}
)

install(TARGETS multiVehicleTool DESTINATION bin)

