cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(MinimalOpenGLSkeleton)

# Add source files
file(GLOB SOURCE_FILES
${CMAKE_SOURCE_DIR}/src/*.c
${CMAKE_SOURCE_DIR}/src/*.cpp)

# Add header files
file(GLOB HEADER_FILES
${CMAKE_SOURCE_DIR}/src/*.h
${CMAKE_SOURCE_DIR}/src/*.hpp)

# GLFW Setup
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)

add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})

# We need a CMAKE_DIR with some code to find external dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

if (WIN32)
find_package(OpenGL REQUIRED)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/glfw-3.3.8)
add_library(GLAD "${CMAKE_SOURCE_DIR}/src/glad_gl.c")

set(LIBS glfw opengl32 GLAD)
elseif (APPLE)
INCLUDE_DIRECTORIES(/System/Library/Frameworks)
add_subdirectory(${CMAKE_SOURCE_DIR}/lib/glfw-3.3.8)
add_library(GLAD "${CMAKE_SOURCE_DIR}/src/glad_gl.c")

FIND_LIBRARY(COCOA_LIBRARY Cocoa)
FIND_LIBRARY(OpenGL_LIBRARY OpenGL)
FIND_LIBRARY(IOKit_LIBRARY IOKit)
FIND_LIBRARY(CoreVideo_LIBRARY CoreVideo)
MARK_AS_ADVANCED(COCOA_LIBRARY OpenGL_LIBRARY)
SET(APPLE_LIBS ${COCOA_LIBRARY} ${IOKit_LIBRARY} ${OpenGL_LIBRARY} ${CoreVideo_LIBRARY})
SET(APPLE_LIBS ${APPLE_LIBS} ${GLFW3_LIBRARY} ${ASSIMP_LIBRARY})

set(LIBS glfw GLAD ${APPLE_LIBS})
else()
set(LIBS )
endif(WIN32)

include_directories(
"${CMAKE_SOURCE_DIR}/src"
"${CMAKE_SOURCE_DIR}/includes"
)

target_link_libraries(${PROJECT_NAME} ${LIBS})