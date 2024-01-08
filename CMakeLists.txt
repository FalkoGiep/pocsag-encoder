cmake_minimum_required(VERSION 3.11-3.18)


project(pocsag)

add_library(pocsag
    src/pocsag.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/include)
