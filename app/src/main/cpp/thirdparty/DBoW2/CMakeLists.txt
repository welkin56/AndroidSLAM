cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)


find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
# 改自上游：SHARED → STATIC。
#
# 上游把它做成独立的 .so，而它 `target_link_libraries(DBoW2 ${OpenCV_LIBS})` 用的是
# **静态** OpenCV —— 于是 **libDBoW2.so 里静态链了一份完整的 cv::**，
# 而 liborb_slam3.so 里**也有一份**。同一个进程里两份 `cv::` 符号。
#
# 后果是真机上的 SIGSEGV：`Frame::ExtractORB` → `ORBextractor` → `cv::GaussianBlur`
# → **`cv::parallel_for_` 被动态链接器解析到了 libDBoW2.so 那一份** ——
# 那是并行后端的分发点，背后带全局状态（这份 OpenCV 开了 TBB 后端），两个副本互相踩。
# 完整调用链见 docs/changes/2026-09-28-m5-slam-on-phone.md。
#
# 静态链进来之后，整个进程只剩一份 OpenCV 的符号，问题消失。
add_library(DBoW2 STATIC ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

