cmake_minimum_required(VERSION 3.24)

project(
    GPU_Traffic_Simulator
    VERSION 0.1.0
    LANGUAGES CXX CUDA
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# CUDA runtime
find_package(CUDAToolkit REQUIRED)

# raylib
include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
)

FetchContent_MakeAvailable(raylib)

add_executable(gpu_traffic_simulator
    src/main.cpp
    src/cuda/cuda_test.cu
    src/simulation/TrafficSimulation.cpp
    src/rendering/CameraController.cpp
)

target_include_directories(gpu_traffic_simulator
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(gpu_traffic_simulator
    PRIVATE
        raylib
        CUDA::cudart
)

set_target_properties(gpu_traffic_simulator PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
)

if(MSVC)
    target_compile_options(gpu_traffic_simulator PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:/W4>
    )
else()
    target_compile_options(gpu_traffic_simulator PRIVATE
        $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wpedantic>
    )
endif()
