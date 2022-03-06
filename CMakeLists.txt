cmake_minimum_required(VERSION 3.0.0)
project(spider_robot VERSION 0.1.0)

# gcc flags
set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -pthread")
set(GCC_COVERAGE_LINK_FLAGS "")

include(CTest)
enable_testing()

include_directories(
    /opt/vc/include 
    /opt/vc/include/interface/vcos/pthreads 
    inc
)
link_directories(/opt/vc/lib)
set(SOURCES 
    src/main.cpp 
    src/servo.cpp 
    src/utils.cpp 
    inc/servo.h 
    inc/utils.h
    )
add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} bcm_host pigpio)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

