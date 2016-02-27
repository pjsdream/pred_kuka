cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})



# application for openni tf recorder
rosbuild_add_executable(openni_recorder
src/openni_recorder.cpp
)
target_link_libraries(openni_recorder pcpred)

# application for benchmark generator
rosbuild_add_executable(benchmark_generator
src/benchmark_generator.cpp
)
target_link_libraries(benchmark_generator pcpred)

# application for learning
rosbuild_add_executable(learn_motion
src/learn_motion.cpp
)
target_link_libraries(learn_motion pcpred)

# application for openni recorded tf publisher
rosbuild_add_executable(playback
src/playback.cpp
)
target_link_libraries(playback pcpred)


# application for Q-learning task planner
rosbuild_add_executable(task_planner
src/task_planner.cpp
)
target_link_libraries(task_planner pcpred)

# application for motion planner
rosbuild_add_executable(motion_planner
src/motion_planner.cpp
src/motion_planner.h
)
target_link_libraries(motion_planner pcpred)
