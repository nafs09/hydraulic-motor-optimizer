cmake_minimum_required(VERSION 3.10)
project(HydraulicMotorOptimizer LANGUAGES CXX)

# Use C++23 as requested
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warning flags (portable)
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Collect sources (reconfigure when files change)
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS
     ${CMAKE_SOURCE_DIR}/src/*.cpp
     ${CMAKE_SOURCE_DIR}/src/*.cxx
)

# Print sources at configure time to help debug missing files
message(STATUS "HydraulicMotorOptimizer: found ${SRC_FILES}")

# Create executable target from collected sources
add_executable(${PROJECT_NAME} ${SRC_FILES})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

# Include headers for the target (project include dir first)
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)

# Optional: set working directory for VS debugger
if (MSVC)
    set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()
