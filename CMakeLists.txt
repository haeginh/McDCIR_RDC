cmake_minimum_required(VERSION 3.9.0)

project(DCIR_RDC)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 99)

#openMP
FIND_PACKAGE(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# libigl
#option(LIBIGL_USE_STATIC_LIBRARY     "Use libIGL as static library" ON)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_EMBREE            "Use embree"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use imgui"           ON)
option(LIBIGL_WITH_TETGEN            "Use TetGen"         ON)
find_package(LIBIGL REQUIRED QUIET)
set(LIBIGL_DIR ${LIBIGL_INCLUDE_DIR}/..)


file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.hh)
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${LIBIGL_DIR}
                    ${LIBIGL_DIR}/external
                    ${LIBIGL_DIR}/external/libigl-imgui/
                    ${LIBIGL_DIR}/external/imgui/examples/
                    ${LIBIGL_DIR}/external/imgui/
                    ${LIBIGL_DIR}/external/glfw/include
                    ${LIBIGL_DIR}/external/glad/include
                    )

add_executable(DCIR_RDC DCIR_RDC.cc ${sources} ${headers})
# Dependencies of this library
target_link_libraries(DCIR_RDC
    igl::core
    igl::opengl
    igl::opengl_glfw_imgui
    igl::embree
    igl::tetgen
    )
