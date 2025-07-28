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

add_executable(serial_test src/main.cpp)
target_include_directories(serial_test PRIVATE ${3RD_INCLUDE_DIRS})
target_link_libraries(serial_test PRIVATE ${3RD_LIBRARIES_DIRS})



install(TARGETS
  serial_test
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


