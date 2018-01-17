cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stereoRecognition)

find_package(PCL 1.8 REQUIRED)

if(PCL_FOUND)
else(PCL_FOUND)
    message(FATAL_ERROR "PCL is needed for the application.  Cannot continue.")
endif(PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS} include lib/yaml-cpp/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)

if(FLYCAPTURE2)
else(FLYCAPTURE2)
    message(FATAL_ERROR "FlyCapture2 is needed for the Point Grey cameras.  Cannot continue.")
endif(FLYCAPTURE2)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    add_definitions(-DUSE_OMP)
else (OPENMP_FOUND)
    message(STATUS "Open MP not found, the pipeline may not run as optimal for parallel tasks.")
endif()

# Warning this will only work for gcc
#SET(GCC_COVERAGE_COMPILE_FLAGS "-m64")

#add_subdirectory(/usr/src/dlib dlib_build)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

if(OpenCV_FOUND)
    # If the package has been found, several variables will
    # be set, you can find the full list with descriptions
    # in the OpenCVConfig.cmake file.
    # Print some message showing some of them
    message(STATUS "OpenCV library status: ")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else(OpenCV_FOUND)
    message(FATAL_ERROR "OpenCV is needed for the application.  Cannot continue.")
endif(OpenCV_FOUND)

if(CMAKE_VERSION VERSION_LESS "2.8.11")
    # Add OpenCV headers location to your include paths
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

# Add performance flags for release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
    add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
    add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)
endif()

find_package(Boost COMPONENTS thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

if(Boost_FOUND)
elseif(Boost_FOUND)
    message(FATAL_ERROR "Boost is needed for the threading used in this application.  Cannot continue.")
endif(Boost_FOUND)

add_executable (pipeline
        src/main.cpp
        include/preprocessor/SIFTKeyPointDetector.h
        include/typedefs.h
        include/pipeline/RecognitionPipeline.h
        include/pipeline/Hist308RecognitionPipeline.h
        include/pipeline/Hist33RecognitionPipeline.h
        include/pipeline/RecognitionPipelineFactory.h
        include/preprocessor/SurfaceNormalEstimator.h
        include/featuredescriptor/VPFHExtractor.h
        include/featuredescriptor/FeatureExtractor.h
        include/preprocessor/KeypointDetector.h
        include/featuredescriptor/FPFHExtractor.h
        src/pipeline/ConfigReader.cpp
        include/pipeline/ConfigReader.h
        include/classifier/SubjectNotFound.h
        src/pipeline/Config.cpp
        include/pipeline/Config.h
        include/classifier/Subject.h
        include/configdefs.h
        include/classifier/IClassifier.h
        include/classifier/KNN.h
        include/preprocessor/filters.h
        include/utils.h
        include/featuredescriptor/CVPFHExtractor.h
        include/preprocessor/GradientIntensityEstimator.h
        src/acquisition/StereoVision.cpp
        src/acquisition/CCFaceDetector.cpp
        include/acquisition/CCFaceDetector.h
        include/acquisition/StereoVision.h
        include/reconstruct.h
        include/classifier/Svm.h include/classifier/NormalBayes.h)

# TODO change hard coded linkage to libyaml
target_link_libraries (pipeline
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${FLYCAPTURE2}
        ${Boost_LIBRARIES}
        #dlib::dlib
        ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/build/libyaml-cpp.a)

# Helper macro to build and link utility libraries
macro (build_util name)
    add_executable (${name} apps/${name}.cpp)
    target_link_libraries (${name} ${PCL_LIBRARIES} ${OpenCV_LIBS})
endmacro()

build_util(regionGrowing)
build_util(viewer)
build_util(faceDetector)
build_util(reconstruct3D)
build_util(generateSyntheticData)
build_util(svm)