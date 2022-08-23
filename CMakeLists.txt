cmake_minimum_required(VERSION 3.16)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
    include(${CMAKE_CURRENT_BINARY_DIR}/conanbuildinfo.cmake)
else ()
    message(FATAL_ERROR "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif ()

list(APPEND CMAKE_MODULE_PATH ${CONAN_TRAACT_RUN_ENV_ROOT}/cmake)

project(${CONAN_PACKAGE_NAME}_PROJECT VERSION ${CONAN_PACKAGE_VERSION})

include(traact_default_library_setup)


set(COMPONENT_KINECT_AZURE_SRC src/KinectAzure.cpp
        #src/KinectAzure.h
        src/KinectUtils.cpp
        src/KinectUtils.h
        #src/KinectAzureColorOutput.cpp
        #src/KinectAzureDepthOutput.cpp
        #src/KinectAzureBodyOutput.cpp
        #src/KinectAzureStaticData.h
        #src/KinectAzureIROutput.cpp
        #src/KinectAzurePlayer.cpp
        #src/KinectAzurePlayer.h
        src/KinectAzureSingleFilePlayer.cpp)


add_library(${CONAN_PACKAGE_NAME} SHARED ${COMPONENT_KINECT_AZURE_SRC})

conan_target_link_libraries(${CONAN_PACKAGE_NAME})

include(traact_default_library_final)