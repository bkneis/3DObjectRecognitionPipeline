cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stereoRecognition)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
##add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
#add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)

add_executable (stereo_rec
        src/KeyPointDetector.cpp
        include/KeyPointDetector.h
        src/main.cpp
#        src/main_old.cpp
        include/feature_estimation.h
        include/typedefs.h
        include/Node.h
        include/RecognitionPipeline.h
        src/RecognitionPipeline.cpp src/SurfaceNormalEstimator.cpp include/SurfaceNormalEstimator.h)

target_link_libraries (stereo_rec ${PCL_LIBRARIES})

