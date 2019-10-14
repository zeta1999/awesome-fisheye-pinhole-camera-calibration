cmake_minimum_required( VERSION 2.8 )
project(ProjectStereoCalib)

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")



set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread  -ggdb  -O0 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread  -ggdb   -O0 -march=native ")

set(OpenCV_DIR "/usr/local/include/opencv3.2.0/share/OpenCV")
find_package( OpenCV REQUIRED )

message( ${OpenCV_VERSION} )
message( ${OpenCV_INSTALL_PATH} )

include_directories( 
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

message( ${OpenCV_INCLUDE_DIRS} )


add_executable( ${PROJECT_NAME} src/calibCoreAlgorithm.cpp src/main_calibrate.cpp )

target_link_libraries( ${PROJECT_NAME}
    ${OpenCV_LIBS}
    jsoncpp
)

message("PROJECT_LIB_DIR : ${PROJECT_LIB_DIR}")
