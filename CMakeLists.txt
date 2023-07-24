cmake_minimum_required(VERSION 3.16)
project(VIR_GUI)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#openMP
FIND_PACKAGE(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

#boost
find_package(Boost REQUIRED COMPONENTS system filesystem)

# libigl
option(LIBIGL_EMBREE           "Build target igl::embree"          ON)
option(LIBIGL_GLFW             "Build target igl::glfw"            ON)
option(LIBIGL_IMGUI            "Build target igl::imgui"           ON)
option(LIBIGL_COPYLEFT_TETGEN  "Build target igl_copyleft::tetgen" ON)
include(libigl)

#include
file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.hh)
file(GLOB plotSource ${PROJECT_SOURCE_DIR}/implot-master/*.cpp)
file(GLOB imgui $ENV{IMGUI_DIR}/*.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/implot-master
                    ${Boost_INCLUDE_DIRS}
                    )

add_executable(${PROJECT_NAME} main.cc ${sources} ${headers} ${plotSource})

target_link_libraries(${PROJECT_NAME} PUBLIC
                     igl::core
                     igl::glfw
                     igl::imgui
                     igl::embree
                     igl_copyleft::tetgen
                     mysqlclient
                     ${Boost_LIBRARIES}
                     )
