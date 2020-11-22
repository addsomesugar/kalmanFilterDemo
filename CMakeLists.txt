cmake_minimum_required(VERSION 3.17)
project(kalmanFilter)
set(CMAKE_CXX_STANDARD 17)
list(APPEND CMAKE_PREFIX_PATH
        /home/hdf/opensource/opencv-4.2.0-build/opencv-4.2.0
        /home/hdf/opensource/eigen-3.3.8-build/eigen-3.3.8)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)



add_executable(kalmanFilter main.cpp)
target_include_directories(kalmanFilter PUBLIC
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})
target_link_libraries(kalmanFilter ${OpenCV_LIBS})