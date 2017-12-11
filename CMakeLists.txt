cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stereoRecognition)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include lib/yaml-cpp/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add performance flags for release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
    add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
    add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)
endif()

add_executable (pipeline
        src/main.cpp
        include/preprocessor/SIFTKeyPointDetector.h
        include/typedefs.h
        include/RecognitionPipeline.h
        include/preprocessor/SurfaceNormalEstimator.h
        include/featuredescriptor/VPFHExtractor.h
        include/featuredescriptor/FeatureExtractor.h
        include/preprocessor/KeypointDetector.h
        include/featuredescriptor/FPFHExtractor.h
        src/ConfigReader.cpp
        include/ConfigReader.h
        src/Config.cpp
        include/Config.h
        include/configdefs.h
        include/classifier/Classifier.h
        include/classifier/KNN.h
        include/preprocessor/filters.h
        include/utils.h
        include/featuredescriptor/CVPFHExtractor.h
        include/preprocessor/GradientIntensityEstimator.h
        include/featuredescriptor/RIFTFeatureExtractor.h)

target_link_libraries (pipeline ${PCL_LIBRARIES} ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/build/libyaml-cpp.a)

# Utility application to view point clouds
add_executable (cloud_viewer src/viewer.cpp)
target_link_libraries (cloud_viewer ${PCL_LIBRARIES})

# Experimental segmentation algorithm binary
add_executable (seg src/regionGrowing.cpp)
target_link_libraries (seg ${PCL_LIBRARIES})

