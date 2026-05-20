cmake_minimum_required(VERSION 3.5)
project(lw_mcp251xfd VERSION 1.0.0 LANGUAGES C)

add_library(lw_mcp251xfd STATIC
    src/mcp251xfd.c
)

target_include_directories(lw_mcp251xfd PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_compile_features(lw_mcp251xfd PUBLIC c_std_99)
