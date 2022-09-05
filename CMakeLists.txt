cmake_minimum_required(VERSION 3.9.0)

project(2_tracker)

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

#openCV
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

#k4a
FIND_PACKAGE(k4a REQUIRED)
FIND_PACKAGE(k4abt REQUIRED)
add_subdirectory(extern)
add_subdirectory(helper)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
# option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
# option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use imgui"           ON)
# option(LIBIGL_WITH_TETGEN            "Use TetGen"         ON)
find_package(LIBIGL REQUIRED QUIET)
set(LIBIGL_DIR ${LIBIGL_INCLUDE_DIR}/..)
set(KINECT_DIR "/home/hurel/codes/Azure-Kinect-Samples/body-tracking-samples")

file(GLOB sources ${PROJECT_SOURCE_DIR}/src/*.cc)
file(GLOB headers ${PROJECT_SOURCE_DIR}/include/*.hh)
include_directories(${PROJECT_SOURCE_DIR}/include
                    ${KINECT_DIR}/sample_helper_includes/
                    ${KINECT_DIR}/sample_helper_libs/window_controller_3d/
                    ${LIBIGL_DIR}/external/
                    ${LIBIGL_DIR}/include/
                    # ${LIBIGL_DIR}/external/libigl-imgui/
                    # ${LIBIGL_DIR}/external/imgui/examples/
                    # ${LIBIGL_DIR}/external/imgui/
                    # ${LIBIGL_DIR}/external/glfw/include
                    # ${LIBIGL_DIR}/external/glad/include
                    )

## 2_tracker
add_executable(2_tracker 2_tracker.cc ${sources} ${headers})
target_link_libraries(2_tracker
    k4a
    #k4arecord
    k4abt
    window_controller_3d::window_controller_3d
    ${OpenCV_LIBS}
    )

## ocr_setting
# add_executable(ocr_setting ocr_setting.cc ${sources} ${headers})
# target_link_libraries(ocr_setting ${OpenCV_LIBS})

## 3_synchronization
#add_executable(3_synchronization 3_synchronization.cc ${sources} ${headers})
#target_link_libraries(3_synchronization k4a ${OpenCV_LIBS})

## 4_camera_calib
#add_executable(4_camera_calib 4_camera_calib.cc ${sources} ${headers})
#target_link_libraries(4_camera_calib k4a ${OpenCV_LIBS})

## 5_charuco_gen
#add_executable(5_charuco_gen 5_charuco_gen.cc)
#target_link_libraries(5_charuco_gen ${OpenCV_LIBS})

