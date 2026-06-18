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
    src/pca9685.cpp
    src/servo_calibration.cpp
    src/utils.cpp 
    inc/pca9685.h
    inc/servo.h 
    inc/servo_calibration.h
    inc/utils.h
    )
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE inc)
target_link_libraries(${PROJECT_NAME} pigpio)

add_executable(spider_diagnostics src/diagnostics.cpp)
target_include_directories(spider_diagnostics PRIVATE inc)
target_link_libraries(spider_diagnostics pigpio)

set(DRY_RUN_SOURCES
    src/motion_command.cpp
    src/gait_controller.cpp
    src/servo_calibration.cpp
)

add_executable(spider_gait_dry_run
    src/gait_dry_run_main.cpp
    ${DRY_RUN_SOURCES}
)
target_include_directories(spider_gait_dry_run PRIVATE inc)

add_executable(test_servo_math
    tests/test_servo_math.cpp
    src/servo_calibration.cpp
)
target_include_directories(test_servo_math PRIVATE inc)
add_test(NAME servo_math COMMAND test_servo_math)

add_executable(test_motion_command
    tests/test_motion_command.cpp
    src/motion_command.cpp
)
target_include_directories(test_motion_command PRIVATE inc)
add_test(NAME motion_command COMMAND test_motion_command)

add_executable(test_gait_controller
    tests/test_gait_controller.cpp
    ${DRY_RUN_SOURCES}
)
target_include_directories(test_gait_controller PRIVATE inc)
add_test(NAME gait_controller COMMAND test_gait_controller)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
