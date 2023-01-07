cmake_minimum_required(VERSION 3.5)
project(E310)
set(CMAKE_BUILD_TYPE "Debug")

execute_process(
    COMMAND echo "copy ini file"
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/detectParams.ini ${PROJECT_BINARY_DIR}/detectParams.ini
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/uavFeature.ini ${PROJECT_BINARY_DIR}/uavFeature.ini
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/package.ini ${PROJECT_BINARY_DIR}/package.ini
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/shield.ini ${PROJECT_BINARY_DIR}/shield.ini
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/tfreq.ini ${PROJECT_BINARY_DIR}/tfreq.ini
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_SOURCE_DIR}/trap.ini ${PROJECT_BINARY_DIR}/trap.ini
)

#添加.h文件
include_directories(./include)
include_directories(./include/TrapTransmit)
include_directories(./include/LabelTransmit)
include_directories(./include/MapTransmit)

#添加.c文件
aux_source_directory(./src SRC_LIST)
aux_source_directory(./src/TrapTransmit SRC_LIST)
aux_source_directory(./src/LabelTransmit SRC_LIST)
aux_source_directory(./src/MapTransmit SRC_LIST)

message(STATUS "${SRC_LIST}" )
add_executable(E310 ${SRC_LIST})

# add_executable(E310 main.cpp ctrDetect.cpp paramRead.cpp readUavParam.cpp sharedFuncs.cpp uavDetect.cpp 
# vidDetect.cpp cwDetect.cpp pscanctrdetect.cpp pscanviddetect.cpp)





# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11 -O0" )

# 寻找OpenCV库
set(OpenCV_INCLUDE_DIRS "/usr/include/opencv4")
find_package( OpenCV 4 REQUIRED )
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} )
message(STATUS "OPENCV_LIB = ${OpenCV_INCLUDE_DIRS}")
# 链接OpenCV库
target_link_libraries( E310 ${OpenCV_LIBS} )


TARGET_LINK_LIBRARIES( E310 pthread iio)
# TARGET_LINK_LIBRARIES( E310 iio)

