cmake_minimum_required(VERSION 3.10)
project(spider_robot VERSION 0.1.0)

# gcc flags
set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -pthread")
set(GCC_COVERAGE_LINK_FLAGS "")

include(CTest)
enable_testing()

set(SOURCES 
    src/main.cpp 
    src/servo.cpp 
    src/utils.cpp 
    inc/servo.h 
    inc/utils.h
    )
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE inc)
target_link_libraries(${PROJECT_NAME} pigpio)

add_executable(spider_diagnostics src/diagnostics.cpp)
target_include_directories(spider_diagnostics PRIVATE inc)
target_link_libraries(spider_diagnostics pigpio)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
