CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(libfreenect REQUIRED)
FIND_PACKAGE(PCL REQUIRED COMPONENTS common visualization)

INCLUDE_DIRECTORIES(${PCL_COMMON_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PCL_VISUALIZATION_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${libfreenect_INCLUDE_DIRS})

ADD_EXECUTABLE(slam_project main.cpp Kinect_Input.cpp PointCloudProcess.cpp Detect.cpp)
TARGET_LINK_LIBRARIES(slam_project ${OpenCV_LIBS} ${FREENECT_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})



