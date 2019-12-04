# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(phenobot_camera)

add_compile_options(-std=c++11)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs cv_bridge image_transport)
find_package(OpenCV)
find_package(Boost REQUIRED filesystem system)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

find_package(OpenCV)

#for json
file(GLOB INCLUDE_FILES "include/json/*.h")
install(FILES ${INCLUDE_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/json)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(/home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/include/Arena/ /home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/include/Save/ /home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/include/GenTL/ /home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/GenICam/library/CPP/include/ ${Boost_INCLUDE_DIRS} )

link_directories(/home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/lib/ /home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/GenICam/library/lib/Linux64_ARM/ /home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/opencv/ /home/nvidia/Downloads/ArenaSDK_v0.1.13_Linux_ARM64/ArenaSDK_Linux_ARM64/ffmpeg/ ${Boost_LIBRARY_DIRS} "/usr/include/jsoncpp" -lb64) 

#add_executable(my_subscriber src/my_subscriber.cpp)
#target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(my_subscriber beginner_tutorials_generate_messages_cpp)

set(SRC src/singleCamera.cpp src/jetsonGPIO.cpp src/lucidPHX.cpp src/paramListener.cpp)
set(SRC_MAIN src/main.cpp)

add_executable(main ${SRC_MAIN} ${SRC})
target_link_libraries(main ${catkin_LIBRARIES})
add_dependencies(main beginner_tutorials_generate_messages_cpp)

target_link_libraries(main -larena -larenac -larenacd -larenad -lgentl -lgentld -lsave -lsavec -lsavecd -lsaved ${OpenCV_LIBS} ${Boost_LIBRARIES}
-lCLAllSerial_gcc48_v3_1     -llog4cpp_gcc48_v3_1
-lCLProtocol_gcc48_v3_1      -lLog_gcc48_v3_1
-lFirmwareUpdate_gcc48_v3_1  -lMathParser_gcc48_v3_1
-lGCBase_gcc48_v3_1          -lNodeMapData_gcc48_v3_1
-lGenApi_gcc48_v3_1          -lXmlParser_gcc48_v3_1 -lopencv_core -lopencv_imgproc -lavcodec -lavdevice -lavfilter -lavformat -lavutil -lswresample -lswscale
-ljsoncpp -lpthread
)




# %EndTag(FULLTEXT)%


