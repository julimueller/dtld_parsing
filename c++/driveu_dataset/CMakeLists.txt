cmake_minimum_required(VERSION 2.8.3)
project(driveu_dataset)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../bin)

find_package(OpenCV 3.2 REQUIRED)
find_package(jsoncpp REQUIRED)
IF(OpenCV_FOUND)
  message("ocv found")
  add_definitions("-DOpenCV_FOUND")
ENDIF()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
add_definitions ("-Wall")

INCLUDE_DIRECTORIES(include)

# lib
SET(SOURCES   src/driveu_dataset.cpp src/driveu_test.cpp src/compand.cpp src/driveu_calibration.cpp src/driveu_vehicle_data.cpp)
SET(HEADERS   include/driveu_dataset.h include/compand.h include/driveu_calibration.h include/driveu_vehicle_data.h)

ADD_EXECUTABLE(driveu_test ${HEADERS} ${SOURCES})
add_library(driveu_dataset SHARED ${HEADERS} ${SOURCES})
target_link_libraries(driveu_dataset ${OpenCV_LIBS} jsoncpp)
target_link_libraries(driveu_test ${OpenCV_LIBS} jsoncpp)

install (TARGETS driveu_dataset ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install (FILES ${HEADERS} DESTINATION "include/driveu_dataset")
INSTALL(TARGETS driveu_test RUNTIME DESTINATION bin)
