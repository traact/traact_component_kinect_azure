cmake_minimum_required(VERSION 3.16)

set(TARGET_NAME traact_component_kinect_azure)

project(${TARGET_NAME} VERSION "0.0.0" LANGUAGES CXX)

find_package(traact_base)
list(APPEND CMAKE_MODULE_PATH ${traact_base_INCLUDE_DIR})

include(traact_default_library_setup)


set(COMPONENT_KINECT_AZURE_SRC src/KinectAzure.cpp
        src/KinectAzure.h
        src/KinectUtils.cpp
        src/KinectUtils.h
        src/KinectAzureOutputColor.cpp
        src/KinectAzureOutputDepth.cpp
        src/KinectAzureOutputBody.cpp
        src/KinectAzureOutputCalibration.cpp
        src/KinectAzureOutputIR.cpp
        #src/KinectAzurePlayer.cpp
        #src/KinectAzurePlayer.h
        src/KinectAzureSingleFilePlayer.cpp)


add_library(${TARGET_NAME} SHARED ${COMPONENT_KINECT_AZURE_SRC})

find_package(traact_spatial REQUIRED)
find_package(traact_vision REQUIRED)
find_package(kinect-azure-sensor-sdk REQUIRED)

if (WITH_BODYTRACKING)
find_package(kinect-azure-bodytracking-sdk REQUIRED)
endif()

target_link_libraries(${TARGET_NAME} PUBLIC traact_spatial::traact_spatial traact_vision::traact_vision kinect-azure-sensor-sdk::kinect-azure-sensor-sdk) 

if(WITH_BODYTRACKING)
    add_definitions(-DWITH_BODYTRACKING)
    target_link_libraries(${TARGET_NAME}  PUBLIC kinect-azure-bodytracking-sdk::kinect-azure-bodytracking-sdk )
endif()

include(traact_default_library_final)