cmake_minimum_required(VERSION 3.10.0)

project(serial_test)


find_package(ament_cmake REQUIRED)

# Set all libraries 3rd party libraries
file(GLOB 3RD_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/3rd/*/include
)
# GLOB tất cả file .so và .a trong thư mục 3rd/
file(GLOB 3RD_LIBRARIES_DIRS "3rd/*/lib/*.so" "3rd/*/lib/*.a")

# In ra danh sách thư viện tìm thấy (debug)
message(STATUS "Found libraries: ${MYLIBS}")

include_directories(
    include
)

add_executable(serial_test  src/main.cpp)
target_include_directories(serial_test PRIVATE ${3RD_INCLUDE_DIRS})
target_link_libraries(serial_test  ${3RD_LIBRARIES_DIRS})

# Set RPATH to find libraries in install directory
set_target_properties(serial_test PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../lib;$ENV{ROS_DISTRO}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)



install(TARGETS
  serial_test
  DESTINATION lib/${PROJECT_NAME}
)

# install 3rd party libraries
install(DIRECTORY 3rd/
  DESTINATION lib/${PROJECT_NAME}
)

# Install shared libraries to system library path
install(FILES 3rd/wit_c_sdk/lib/libwit_c_sdk.so 
  DESTINATION lib
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(DIRECTORY include
  DESTINATION include
)

ament_package()


