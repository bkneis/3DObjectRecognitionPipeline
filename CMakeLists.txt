cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stereoRecognition)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} include lib/yaml-cpp/include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_definitions(-Wall -O3 -DNDEBUG -pipe -ffast-math -funroll-loops -ftree-vectorize -fomit-frame-pointer -pipe -mfpmath=sse -mmmx -msse -mtune=core2 -march=core2 -msse2 -msse3 -mssse3 -msse4)
##add_definitions(-momit-leaf-frame-pointer -fomit-frame-pointer -floop-block -ftree-loop-distribution -ftree-loop-linear -floop-interchange -floop-strip-mine -fgcse-lm -fgcse-sm -fsched-spec-load)
#add_definitions (-Wall -O3 -Winvalid-pch -pipe -funroll-loops -fno-strict-aliasing)

add_executable (stereo_rec
        include/preprocessor/SIFTKeyPointDetector.h
        src/main.cpp
        include/feature_estimation.h
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
        include/config_mappings.h
        src/ParameterBuilder.cpp
        include/ParameterBuilder.h
        include/classifier/KNN.h
        include/classifier/Classifier.h include/loader.h include/featuredescriptor/CVPFHExtractor.h include/preprocessor/GradientIntensityEstimator.h include/featuredescriptor/RIFTFeatureExtractor.h)

target_link_libraries (stereo_rec ${PCL_LIBRARIES} ${CMAKE_SOURCE_DIR}/lib/yaml-cpp/build/libyaml-cpp.a)

