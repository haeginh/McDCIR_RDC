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

#k4a
FIND_PACKAGE(k4a REQUIRED)
FIND_PACKAGE(k4abt REQUIRED)
add_subdirectory(extern)
add_subdirectory(helper)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use imgui"           ON)
option(LIBIGL_WITH_TETGEN            "Use TetGen"         ON)
find_package(LIBIGL REQUIRED QUIET)
link_directories(/home/hurel/codes/libigl/v2.2.0/lib/)

## trase
#find_package(trase REQUIRED)

file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB g4sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB g4headers ${PROJECT_SOURCE_DIR}/include/*.hh)
include_directories(${PROJECT_SOURCE_DIR}/include
                    ~/Azure-Kinect-Samples/body-tracking-samples/sample_helper_includes/
#                    ~/codes/trase/src/
                    ~/codes/libigl/v2.2.0/external/
                    ~/codes/libigl/v2.2.0/include/
                    ~/codes/libigl/v2.2.0/external/libigl-imgui/
                    ~/codes/libigl/v2.2.0/external/imgui/examples/
                    ~/codes/libigl/v2.2.0/external/imgui/
                    ~/codes/libigl/v2.2.0/external/glfw/include
                    ~/codes/libigl/v2.2.0/external/glad/include
                    )

add_executable(RemoteCal RemoteCal.cc ${sources} ${headers})
add_executable(MapCalculator calculator.cc ${sources} ${headers} ${g4sources} ${g4headers})

# Dependencies of this library
target_link_libraries(RemoteCal
    igl::core
    igl::opengl
#    igl::opengl_glfw
#    /usr/lib/libglfw3.a
    /home/hurel/codes/libigl/v2.2.0/lib/libglfw3.a
    igl::opengl_glfw_imgui
    igl::tetgen
    k4a
    k4arecord
    k4abt
    window_controller_3d::window_controller_3d
#    glfw
#    trase
    ${Geant4_LIBRARIES}
    /home/hurel/codes/libigl/v2.2.0/lib/libglfw3.a
    )
target_link_libraries(MapCalculator ${Geant4_LIBRARIES})

