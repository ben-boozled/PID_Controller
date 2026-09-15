cmake_minimum_required(VERSION 3.10.0)
project(PID_Controller VERSION 0.1.0 LANGUAGES C)

set(SOURCE_FILES
    src/main.c
    src/PID.c
    src/altitude_sim.c)

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

target_include_directories(${PROJECT_NAME} PUBLIC include)