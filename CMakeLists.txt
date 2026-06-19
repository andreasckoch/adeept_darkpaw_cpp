cmake_minimum_required(VERSION 3.10)
project(spider_robot VERSION 0.1.0)

# gcc flags
set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -pthread")
set(GCC_COVERAGE_LINK_FLAGS "")

include(CTest)
enable_testing()

set(HAL_SOURCES
    src/hal/pca9685.cpp
    inc/pca9685.h
)

set(ACTUATION_SOURCES
    src/actuation/servo.cpp
    src/actuation/servo_calibration.cpp
    inc/servo.h
    inc/servo_calibration.h
)

set(COMMON_SOURCES
    src/common/utils.cpp
    inc/utils.h
)

set(SOURCES
    src/tools/spider_robot_main.cpp
    ${HAL_SOURCES}
    ${ACTUATION_SOURCES}
    ${COMMON_SOURCES}
)
add_executable(${PROJECT_NAME} ${SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE inc)
target_link_libraries(${PROJECT_NAME} pigpio)

add_executable(spider_diagnostics src/tools/diagnostics_main.cpp)
target_include_directories(spider_diagnostics PRIVATE inc)
target_link_libraries(spider_diagnostics pigpio)

set(DRY_RUN_SOURCES
    src/gait/motion_command.cpp
    src/gait/gait_controller.cpp
    src/actuation/servo_calibration.cpp
)

set(GAIT_AUTHORING_SOURCES
    src/gait/gait_json.cpp
    src/gait/gait_pose.cpp
    src/gait/gait_definition.cpp
    src/gait/gait_trajectory.cpp
    src/actuation/servo_calibration.cpp
)

add_executable(spider_gait_dry_run
    src/tools/gait_dry_run_main.cpp
    ${DRY_RUN_SOURCES}
)
target_include_directories(spider_gait_dry_run PRIVATE inc)

add_executable(spider_validate_gait
    src/tools/gait_validate_main.cpp
    ${GAIT_AUTHORING_SOURCES}
)
target_include_directories(spider_validate_gait PRIVATE inc)

add_executable(spider_compile_gait
    src/tools/gait_compile_main.cpp
    ${GAIT_AUTHORING_SOURCES}
)
target_include_directories(spider_compile_gait PRIVATE inc)

add_executable(spider_replay_gait
    src/tools/gait_replay_main.cpp
    ${GAIT_AUTHORING_SOURCES}
)
target_include_directories(spider_replay_gait PRIVATE inc)

add_executable(spider_play_gait
    src/tools/gait_play_main.cpp
    ${GAIT_AUTHORING_SOURCES}
    ${HAL_SOURCES}
)
target_include_directories(spider_play_gait PRIVATE inc)
target_link_libraries(spider_play_gait pigpio)

add_executable(test_servo_math
    tests/test_servo_math.cpp
    src/actuation/servo_calibration.cpp
)
target_include_directories(test_servo_math PRIVATE inc)
add_test(NAME servo_math COMMAND test_servo_math)

add_executable(test_motion_command
    tests/test_motion_command.cpp
    src/gait/motion_command.cpp
)
target_include_directories(test_motion_command PRIVATE inc)
add_test(NAME motion_command COMMAND test_motion_command)

add_executable(test_gait_controller
    tests/test_gait_controller.cpp
    ${DRY_RUN_SOURCES}
)
target_include_directories(test_gait_controller PRIVATE inc)
add_test(NAME gait_controller COMMAND test_gait_controller)

add_executable(test_gait_authoring
    tests/test_gait_authoring.cpp
    ${GAIT_AUTHORING_SOURCES}
)
target_include_directories(test_gait_authoring PRIVATE inc)
add_test(NAME gait_authoring COMMAND test_gait_authoring)

add_executable(test_gait_examples
    tests/test_gait_examples.cpp
    ${GAIT_AUTHORING_SOURCES}
)
target_include_directories(test_gait_examples PRIVATE inc)
target_compile_definitions(test_gait_examples PRIVATE REPO_ROOT="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME gait_examples COMMAND test_gait_examples)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}")

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
