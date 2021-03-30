cmake_minimum_required(VERSION 3.9.0)

project(RemoteCal)

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

#Geant4
option(WITH_GEANT4_UIVIS "Build example with Geant4 UI and Vis drivers" ON)
if(WITH_GEANT4_UIVIS)
  find_package(Geant4 REQUIRED ui_all vis_all)
else()
  find_package(Geant4 REQUIRED)
endif()

# libigl
#option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
#option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
#option(LIBIGL_WITH_TETGEN            "Use TetGen"         ON)
find_package(LIBIGL REQUIRED QUIET)

file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB g4sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB g4headers ${PROJECT_SOURCE_DIR}/include/*.hh)
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${PROJECT_SOURCE_DIR}/libigl/include
                    ${PROJECT_SOURCE_DIR}/libigl/external/eigen)
add_executable(Calculator calculator.cc ${sources} ${headers} ${g4sources} ${g4headers})

# Dependencies of this library
target_link_libraries(Calculator ${Geant4_LIBRARIES})


