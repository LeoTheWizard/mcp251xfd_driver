cmake_minimum_required(VERSION 3.13)
project(lw_mcp251xfd VERSION 1.0.0 LANGUAGES C)

add_library(lw_mcp251xfd STATIC
    src/mcp251xfd.c
)

# Namespaced alias so consumers can write `lw_mcp251xfd::lw_mcp251xfd`
# whether they pull us in via add_subdirectory, FetchContent, or find_package.
add_library(lw_mcp251xfd::lw_mcp251xfd ALIAS lw_mcp251xfd)

target_include_directories(lw_mcp251xfd PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(lw_mcp251xfd PUBLIC c_std_99)

target_compile_definitions(lw_mcp251xfd PUBLIC
    MCP251XFD_ENABLE_ERROR_MESSAGES
)
