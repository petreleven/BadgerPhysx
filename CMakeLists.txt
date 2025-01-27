cmake_minimum_required(VERSION 3.5)
project(Badger VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

# AddressSanitizer flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address ")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fsanitize=address")


add_executable(Main src/main.cpp)
add_subdirectory(include)

target_include_directories(Main PUBLIC
                          "${CMAKE_BINARY_DIR}"
                          "${CMAKE_SOURCE_DIR}"
                          "${CMAKE_SOURCE_DIR}/include")

target_link_libraries(Main PUBLIC 
    MathFunctions 
    RigidBody 
    ForceGenerator 
    Core 
    raylib
)

