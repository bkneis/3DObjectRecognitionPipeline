# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/arthur/Downloads/clion-2017.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/arthur/Downloads/clion-2017.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/arthur/iccv2011

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/arthur/iccv2011/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/test_keypoints.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_keypoints.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_keypoints.dir/flags.make

CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o: CMakeFiles/test_keypoints.dir/flags.make
CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o: ../src/test_keypoints.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/arthur/iccv2011/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o -c /home/arthur/iccv2011/src/test_keypoints.cpp

CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/arthur/iccv2011/src/test_keypoints.cpp > CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.i

CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/arthur/iccv2011/src/test_keypoints.cpp -o CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.s

CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.requires:

.PHONY : CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.requires

CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.provides: CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.requires
	$(MAKE) -f CMakeFiles/test_keypoints.dir/build.make CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.provides.build
.PHONY : CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.provides

CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.provides.build: CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o


# Object files for target test_keypoints
test_keypoints_OBJECTS = \
"CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o"

# External object files for target test_keypoints
test_keypoints_EXTERNAL_OBJECTS =

test_keypoints: CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o
test_keypoints: CMakeFiles/test_keypoints.dir/build.make
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_system.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_thread.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_regex.so
test_keypoints: /usr/local/lib/libpcl_common.so
test_keypoints: /usr/local/lib/libpcl_octree.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libpython2.7.so
test_keypoints: /usr/local/lib/libvtkWrappingTools-8.1.a
test_keypoints: /usr/local/lib/libpcl_io.so
test_keypoints: /usr/local/lib/libflann_cpp_s.a
test_keypoints: /usr/local/lib/libpcl_kdtree.so
test_keypoints: /usr/local/lib/libpcl_search.so
test_keypoints: /usr/local/lib/libpcl_surface.so
test_keypoints: /usr/local/lib/libpcl_sample_consensus.so
test_keypoints: /usr/local/lib/libpcl_filters.so
test_keypoints: /usr/local/lib/libpcl_features.so
test_keypoints: /usr/local/lib/libpcl_visualization.so
test_keypoints: /usr/local/lib/libpcl_ml.so
test_keypoints: /usr/local/lib/libpcl_segmentation.so
test_keypoints: /usr/local/lib/libpcl_people.so
test_keypoints: /usr/local/lib/libpcl_registration.so
test_keypoints: /usr/local/lib/libpcl_stereo.so
test_keypoints: /usr/local/lib/libpcl_outofcore.so
test_keypoints: /usr/local/lib/libpcl_recognition.so
test_keypoints: /usr/local/lib/libpcl_keypoints.so
test_keypoints: /usr/local/lib/libpcl_tracking.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_system.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_thread.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libboost_regex.so
test_keypoints: /usr/local/lib/libflann_cpp_s.a
test_keypoints: /usr/lib/x86_64-linux-gnu/libpython2.7.so
test_keypoints: /usr/local/lib/libvtkIOInfovis-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingContextOpenGL2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkTestingRendering-8.1.so.1
test_keypoints: /usr/local/lib/libvtkViewsContext2D-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersGeneric-8.1.so.1
test_keypoints: /usr/local/lib/libvtkTestingGenericBridge-8.1.so.1
test_keypoints: /usr/local/lib/libvtkDomainsChemistryOpenGL2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOAMR-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOExodus-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingVolumeOpenGL2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersFlowPaths-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersHyperTree-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingStencil-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersParallelDIY2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersParallelGeometry-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOParallelXML-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersParallelImaging-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersParallelMPI-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersParallelVerdict-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersPoints-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersProgrammable-8.1.so.1
test_keypoints: /usr/local/lib/libvtkWrappingTools-8.1.a
test_keypoints: /usr/local/lib/libvtkFiltersPython-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersSMP-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersSelection-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersTexture-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersTopology-8.1.so.1
test_keypoints: /usr/local/lib/libvtkViewsGeovis-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOEnSight-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOExportOpenGL2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkInteractionImage-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOImport-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOLSDyna-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOMINC-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOMPIImage-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOMPIParallel-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOMovie-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOPLY-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOParallelNetCDF-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOSQL-8.1.so.1
test_keypoints: /usr/local/lib/libvtkTestingIOSQL-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOTecplotTable-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOVideo-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingStatistics-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingImage-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingMorphological-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingLOD-8.1.so.1
test_keypoints: /usr/local/lib/libvtkParallelMPI4Py-8.1.so.1
test_keypoints: /usr/local/lib/libvtkWebCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkWrappingJava-8.1.so.1
test_keypoints: /usr/local/lib/libpcl_common.so
test_keypoints: /usr/local/lib/libpcl_octree.so
test_keypoints: /usr/local/lib/libpcl_io.so
test_keypoints: /usr/local/lib/libpcl_kdtree.so
test_keypoints: /usr/local/lib/libpcl_search.so
test_keypoints: /usr/local/lib/libpcl_surface.so
test_keypoints: /usr/local/lib/libpcl_sample_consensus.so
test_keypoints: /usr/local/lib/libpcl_filters.so
test_keypoints: /usr/local/lib/libpcl_features.so
test_keypoints: /usr/local/lib/libpcl_visualization.so
test_keypoints: /usr/local/lib/libpcl_ml.so
test_keypoints: /usr/local/lib/libpcl_segmentation.so
test_keypoints: /usr/local/lib/libpcl_people.so
test_keypoints: /usr/local/lib/libpcl_registration.so
test_keypoints: /usr/local/lib/libpcl_stereo.so
test_keypoints: /usr/local/lib/libpcl_outofcore.so
test_keypoints: /usr/local/lib/libpcl_recognition.so
test_keypoints: /usr/local/lib/libpcl_keypoints.so
test_keypoints: /usr/local/lib/libpcl_tracking.so
test_keypoints: /usr/local/lib/libvtklibxml2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkDomainsChemistry-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersAMR-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingMath-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersVerdict-8.1.so.1
test_keypoints: /usr/local/lib/libvtkverdict-8.1.so.1
test_keypoints: /usr/local/lib/libvtkWrappingPython27Core-8.1.so.1
test_keypoints: /usr/local/lib/libvtkGeovisCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkproj4-8.1.so.1
test_keypoints: /usr/local/lib/libvtkViewsInfovis-8.1.so.1
test_keypoints: /usr/local/lib/libvtkChartsCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkViewsCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersImaging-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingLabel-8.1.so.1
test_keypoints: /usr/local/lib/libvtkInfovisLayout-8.1.so.1
test_keypoints: /usr/local/lib/libvtkInfovisCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOParallel-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOGeometry-8.1.so.1
test_keypoints: /usr/local/lib/libvtkexoIIc-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersParallel-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIONetCDF-8.1.so.1
test_keypoints: /usr/local/lib/libvtknetcdfcpp-8.1.so.1
test_keypoints: /usr/local/lib/libvtkjsoncpp-8.1.so.1
test_keypoints: /usr/local/lib/libvtkoggtheora-8.1.so.1
test_keypoints: /usr/local/lib/libvtkNetCDF-8.1.so.1
test_keypoints: /usr/local/lib/libvtkhdf5_hl-8.1.so.1
test_keypoints: /usr/local/lib/libvtkhdf5-8.1.so.1
test_keypoints: /usr/local/lib/libvtksqlite-8.1.so.1
test_keypoints: /usr/lib/x86_64-linux-gnu/libpython2.7.so
test_keypoints: /usr/local/lib/libvtkParallelMPI-8.1.so.1
test_keypoints: /usr/local/lib/libvtkParallelCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOLegacy-8.1.so.1
test_keypoints: /usr/local/lib/libvtkWebGLExporter-8.1.so.1
test_keypoints: /usr/local/lib/libvtkInteractionWidgets-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersHybrid-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersModeling-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingGeneral-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingSources-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingHybrid-8.1.so.1
test_keypoints: /usr/local/lib/libvtkInteractionStyle-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersExtraction-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersStatistics-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingFourier-8.1.so.1
test_keypoints: /usr/local/lib/libvtkalglib-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingAnnotation-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingColor-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingVolume-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOXML-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOXMLParser-8.1.so.1
test_keypoints: /usr/local/lib/libvtkexpat-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOExport-8.1.so.1
test_keypoints: /usr/local/lib/libvtkImagingCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingContext2D-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingFreeType-8.1.so.1
test_keypoints: /usr/local/lib/libvtkfreetype-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtklz4-8.1.so.1
test_keypoints: /usr/local/lib/libvtkIOImage-8.1.so.1
test_keypoints: /usr/local/lib/libvtkDICOMParser-8.1.so.1
test_keypoints: /usr/local/lib/libvtkmetaio-8.1.so.1
test_keypoints: /usr/local/lib/libvtkpng-8.1.so.1
test_keypoints: /usr/local/lib/libvtktiff-8.1.so.1
test_keypoints: /usr/local/lib/libvtkzlib-8.1.so.1
test_keypoints: /usr/local/lib/libvtkjpeg-8.1.so.1
test_keypoints: /usr/lib/x86_64-linux-gnu/libm.so
test_keypoints: /usr/local/lib/libvtkRenderingGL2PSOpenGL2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingOpenGL2-8.1.so.1
test_keypoints: /usr/local/lib/libvtkRenderingCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonColor-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersGeometry-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersSources-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersGeneral-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonComputationalGeometry-8.1.so.1
test_keypoints: /usr/local/lib/libvtkFiltersCore-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonExecutionModel-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonDataModel-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonMisc-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonSystem-8.1.so.1
test_keypoints: /usr/local/lib/libvtksys-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonTransforms-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonMath-8.1.so.1
test_keypoints: /usr/local/lib/libvtkglew-8.1.so.1
test_keypoints: /usr/lib/x86_64-linux-gnu/libSM.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libICE.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libX11.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libXext.so
test_keypoints: /usr/lib/x86_64-linux-gnu/libXt.so
test_keypoints: /usr/local/lib/libvtkgl2ps-8.1.so.1
test_keypoints: /usr/local/lib/libvtklibharu-8.1.so.1
test_keypoints: /usr/local/lib/libvtkCommonCore-8.1.so.1
test_keypoints: CMakeFiles/test_keypoints.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/arthur/iccv2011/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable test_keypoints"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_keypoints.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_keypoints.dir/build: test_keypoints

.PHONY : CMakeFiles/test_keypoints.dir/build

CMakeFiles/test_keypoints.dir/requires: CMakeFiles/test_keypoints.dir/src/test_keypoints.cpp.o.requires

.PHONY : CMakeFiles/test_keypoints.dir/requires

CMakeFiles/test_keypoints.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_keypoints.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_keypoints.dir/clean

CMakeFiles/test_keypoints.dir/depend:
	cd /home/arthur/iccv2011/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/arthur/iccv2011 /home/arthur/iccv2011 /home/arthur/iccv2011/cmake-build-debug /home/arthur/iccv2011/cmake-build-debug /home/arthur/iccv2011/cmake-build-debug/CMakeFiles/test_keypoints.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_keypoints.dir/depend

