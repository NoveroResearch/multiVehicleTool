if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 3.0)
	set(BLUEZ_BUILD_ALWAYS BUILD_ALWAYS 1)
endif()

ExternalProject_Add(bluez
	URL https://www.kernel.org/pub/linux/bluetooth/bluez-5.41.tar.gz
	PREFIX ${PROJECT_SOURCE_DIR}/dep/bluez
	INSTALL_COMMAND ""
	CONFIGURE_COMMAND ${CMAKE_COMMAND} ${PROJECT_SOURCE_DIR}/dep/bluez/src/bluez/ -DCMAKE_MODULE_PATH=${PROJECT_SOURCE_DIR}/cmake/modules
	PATCH_COMMAND ${CMAKE_COMMAND} -E create_symlink ${PROJECT_SOURCE_DIR}/cmake/bluez-CMakeLists.txt ${PROJECT_SOURCE_DIR}/dep/bluez/src/bluez/CMakeLists.txt
	${BLUEZ_BUILD_ALWAYS}
)

unset(BLUEZ_BUILD_ALWAYS)

ExternalProject_Get_Property(bluez BINARY_DIR SOURCE_DIR)
set(bluez_SOURCE_DIR ${SOURCE_DIR})
set(bluez_BINARY_DIR ${BINARY_DIR})
unset(SOURCE_DIR)
unset(BINARY_DIR)

set(BLUEZ_INCLUDE_DIRS ${bluez_SOURCE_DIR}/ ${bluez_SOURCE_DIR}/include)
set(BLUEZ_LIBRARIES ${bluez_BINARY_DIR}/libbluez.a)
