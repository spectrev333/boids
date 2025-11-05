cmake_minimum_required(VERSION 4.0)
project(boids_execise)
set(CMAKE_C_STANDARD 99)

include(FetchContent)

# Downloads prebuild from github releases
FetchContent_Declare(
        raylib
        URL "https://github.com/raysan5/raylib/releases/download/5.5/raylib-5.5_linux_amd64.tar.gz"
)
FetchContent_MakeAvailable(raylib)

# Set the include and lib locations as in the unpacked archive
set(RAYLIB_INCLUDE_DIR "${raylib_SOURCE_DIR}/include")
set(RAYLIB_LIB_DIR "${raylib_SOURCE_DIR}/lib")

# Add the target as a library setting it's properties
add_library(raylib_shared SHARED IMPORTED)
set_target_properties(raylib_shared PROPERTIES
    IMPORTED_LOCATION "${RAYLIB_LIB_DIR}/libraylib.so"
    INTERFACE_INCLUDE_DIRECTORIES "${RAYLIB_INCLUDE_DIR}"
)

# Regular cmake stuff
add_executable(boids_execise main.c)
target_link_libraries(boids_execise raylib_shared)
