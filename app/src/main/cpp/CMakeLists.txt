# 把 ORB-SLAM3 交叉编译成 liborb_slam3.so，与 JNI 层一起打进 APK。
#
# 依赖的分工与对第三方副本的移植改动见 thirdparty/README.md，
# 移植理由见 docs/changes/2026-09-27-m3-cross-compile-orb-slam3.md。
# 自带的是 ORB-SLAM3 fork 过、版本敏感的小库（DBoW2 / g2o / Sophus），
# 外装的是通用、体积大的（OpenCV / Eigen / Boost）。
cmake_minimum_required(VERSION 3.22.1)
# 并行编译数上限：AGP 调 ninja 不带 -j，默认按核数开满，容易撑爆内存。
# 必须在 project() 之前设置，写在后面不生效。
set(CMAKE_JOB_POOLS compile=8)
set(CMAKE_JOB_POOL_COMPILE compile)
# 必须在任何 target 创建之前：该变量在创建 target 时用来初始化
# EXPORT_COMPILE_COMMANDS 属性，写在后面不生效。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 必须带 C：g2o 里有 os_specific.c，只开 CXX 会在 configure 阶段报没有 C 编译器。
project(slam_jni LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 必须在任何 target 创建之前。AGP 的 Debug 既不传 -O 也不定义 NDEBUG，而 PC 上是 Release。
# 缺这两项会让单帧耗时触发 ORB-SLAM3 的时间戳跳变守卫，还会让 assert 打崩 App。
add_compile_options(-O3 -DNDEBUG)

set(CPP_DIR  ${CMAKE_CURRENT_LIST_DIR})
set(TP_DIR   ${CPP_DIR}/thirdparty)
set(ORB_ROOT ${TP_DIR}/orb_slam3)

# ---- OpenCV ----
# 外装：通用、成熟、体积大（872 MB），官方有预编译包，不放进仓库。
# 默认放在 thirdparty/ 下，换位置用 -DOPENCV_ANDROID_SDK=... 指定。
# SDK 内部的目录结构不能动：OpenCVConfig.cmake 靠自身位置往上推算 SDK 根，挪了就找不到库。
get_filename_component(_opencv_default ${TP_DIR}/OpenCV-android-sdk ABSOLUTE)
set(OPENCV_ANDROID_SDK ${_opencv_default} CACHE PATH "OpenCV 官方 Android SDK 的根目录")
set(OpenCV_DIR ${OPENCV_ANDROID_SDK}/sdk/native/jni)
# 它靠 ANDROID_NDK_ABI_NAME 查找 abi-<abi> 子目录，而 NDK toolchain 只设 ANDROID_ABI。
# 不显式指定会去找一个不存在的目录，报 ABI 不兼容。
set(OpenCV_CONFIG_SUBDIR /abi-${ANDROID_ABI})
# 只点名用到的模块。全量列表里带 opencv_java（Java 绑定，18 MB），用不上。
find_package(OpenCV REQUIRED core imgproc imgcodecs features2d calib3d flann)

# ---- Boost ----
# 躲不掉：12 个文件里的 serialize() 没有 #ifdef 保护，不存地图也要链上。
# Boost 没有官方安卓发布物，用 tools/setup-boost-android.sh 自编。
set(BOOST_ROOT_ANDROID ${TP_DIR}/boost-android CACHE PATH "安卓版 Boost 的安装前缀")
# 路径写死，不用 find_library：NDK toolchain 把 find_* 的搜索根锁在 sysroot 里，
# PATHS 给的目录会被重挂到 sysroot 下，结果装在哪都找不到。
# setup 脚本用 --layout=system，产物名固定。
set(BOOST_SERIALIZATION_LIB ${BOOST_ROOT_ANDROID}/lib/libboost_serialization.a)
if(NOT EXISTS ${BOOST_SERIALIZATION_LIB})
    message(FATAL_ERROR
        "找不到安卓版 Boost.Serialization。先跑 tools/setup-boost-android.sh，"
        "或加 -DBOOST_ROOT_ANDROID=<装在哪>。找的是：${BOOST_SERIALIZATION_LIB}")
endif()

# ---- Eigen ----
# 外装，不自带：Eigen 通用、API 稳定、纯头文件，系统包（apt libeigen3-dev）即可。
# 与 g2o/DBoW2/Sophus 不同，后三个是 ORB-SLAM3 fork 过、版本敏感的，必须跟着代码走。
# 难点在找得到：NDK toolchain 把 find_* 的搜索根锁在 sysroot 里，会无视系统装的 Eigen，
# NO_CMAKE_FIND_ROOT_PATH 用来绕开。
find_path(EIGEN3_CONFIG_DIR Eigen3Config.cmake
          PATHS /usr/share/eigen3/cmake /usr/lib/cmake/eigen3 /usr/local/share/eigen3/cmake
          NO_CMAKE_FIND_ROOT_PATH)
if(NOT EIGEN3_CONFIG_DIR)
    message(FATAL_ERROR "找不到系统的 Eigen3。装一下：sudo apt install libeigen3-dev")
endif()
set(Eigen3_DIR ${EIGEN3_CONFIG_DIR} CACHE PATH "系统装的 Eigen3 的 cmake 配置目录")
# 先自己找一次：EIGEN3_INCLUDE_DIR 由官方配置算出，结果进缓存后，
# g2o 内部的 FIND_PACKAGE(Eigen3) 直接命中，不会再去 sysroot 里找。
find_package(Eigen3 3.1.0 REQUIRED)

include_directories(
    ${BOOST_ROOT_ANDROID}/include
    ${EIGEN3_INCLUDE_DIR}
    ${ORB_ROOT}
    ${ORB_ROOT}/include
    ${ORB_ROOT}/include/CameraModels
    ${TP_DIR}/DBoW2          # 头文件里写的是 "DBoW2/BowVector.h"、"DUtils/Random.h"
    ${TP_DIR}/g2o            # "g2o/core/block_solver.h"
    ${TP_DIR}/Sophus         # "sophus/geometry.hpp"
)

# ---- DBoW2 / g2o ----
add_subdirectory(${TP_DIR}/DBoW2 ${CMAKE_BINARY_DIR}/DBoW2)
add_subdirectory(${TP_DIR}/g2o   ${CMAKE_BINARY_DIR}/g2o)

# 这两家的 CMakeLists 把产物写到各自源码树的 lib/ 下（上游给 PC 版用）。
# 用 target 属性改到构建目录，属性优先于旧的 LIBRARY_OUTPUT_PATH 变量。
set(_slam_out ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
if(NOT _slam_out)
    set(_slam_out ${CMAKE_BINARY_DIR})
endif()
set_target_properties(DBoW2 g2o PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${_slam_out}
    ARCHIVE_OUTPUT_DIRECTORY ${_slam_out})

# ---- ORB-SLAM3 本体（Viewer / MapDrawer 已是空壳）----
# orb_track.cpp 是我们写的，编在这个 target 里：它要构造 cv::Mat，而 slam_jni 刻意不链 OpenCV，
# 否则进程里会出现两份 cv:: 全局状态。详见 docs/changes/2026-09-28-m5-slam-on-phone.md。
add_library(orb_slam3 SHARED
    ${CPP_DIR}/md5.cpp                # 替代上游依赖的 -lcrypto
    ${CPP_DIR}/orb_track.cpp          # 我们写的，编在这里的原因见上
    ${CPP_DIR}/measure_center.cpp     # 同上，它用到 cv::KeyPoint 和 MapPoint
    ${CPP_DIR}/calib.cpp              # 同上，标定用的是 OpenCV calib3d
    ${ORB_ROOT}/src/System.cc
    ${ORB_ROOT}/src/Tracking.cc
    ${ORB_ROOT}/src/LocalMapping.cc
    ${ORB_ROOT}/src/LoopClosing.cc
    ${ORB_ROOT}/src/ORBextractor.cc
    ${ORB_ROOT}/src/ORBmatcher.cc
    ${ORB_ROOT}/src/FrameDrawer.cc
    ${ORB_ROOT}/src/Converter.cc
    ${ORB_ROOT}/src/MapPoint.cc
    ${ORB_ROOT}/src/KeyFrame.cc
    ${ORB_ROOT}/src/Atlas.cc
    ${ORB_ROOT}/src/Map.cc
    ${ORB_ROOT}/src/MapDrawer.cc
    ${ORB_ROOT}/src/Optimizer.cc
    ${ORB_ROOT}/src/Frame.cc
    ${ORB_ROOT}/src/KeyFrameDatabase.cc
    ${ORB_ROOT}/src/Sim3Solver.cc
    ${ORB_ROOT}/src/Viewer.cc
    ${ORB_ROOT}/src/ImuTypes.cc
    ${ORB_ROOT}/src/G2oTypes.cc
    ${ORB_ROOT}/src/OptimizableTypes.cpp
    ${ORB_ROOT}/src/MLPnPsolver.cpp
    ${ORB_ROOT}/src/GeometricTools.cc
    ${ORB_ROOT}/src/TwoViewReconstruction.cc
    ${ORB_ROOT}/src/Config.cc
    ${ORB_ROOT}/src/Settings.cc
    ${ORB_ROOT}/src/CameraModels/Pinhole.cpp
    ${ORB_ROOT}/src/CameraModels/KannalaBrandt8.cpp
)

# md5.h 只给 orb_slam3 用（System.cc 里 include）。不放进全局 include_directories：
# target 级范围更窄，不会漏给 g2o、DBoW2、slam_jni。
target_include_directories(orb_slam3 PRIVATE ${CPP_DIR})

# 全部 PRIVATE，尤其是 DBoW2。DBoW2 是静态库，它的 CMakeLists 用默认签名（PUBLIC）
# 链着静态 OpenCV；这里若用 PUBLIC，OpenCV 会一路传到 slam_jni，进程里就有两份 cv::
# 及其背后的全局状态，真机上会崩。DBoW2 的代码已在 liborb_slam3.so 里，slam_jni 不必再链。
target_link_libraries(orb_slam3 PRIVATE
    DBoW2 g2o
    ${OpenCV_LIBS}
    ${BOOST_SERIALIZATION_LIB}
    z log)

# ---- JNI 层 ----
# 链上 orb_slam3：JNI 层要用它，同时 liborb_slam3.so 的静态初始化会在 App 启动时跑一遍。
add_library(slam_jni SHARED
    jni_bridge.cpp        # JNI 符号层：翻译 + 转发
    log_redirect.cpp      # 基础设施：把 fd 1/2 接进 logcat
    slam_engine.cpp       # 构造并持有 ORB_SLAM3::System
    )
# 只给头文件路径，不链库。ImuTypes.h / System.h 自己 include OpenCV，编译时要能找到；
# 但头文件不产生符号引用，链了才会，链了进程里就会出现第二份 cv:: 实现。
target_include_directories(slam_jni PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(slam_jni PRIVATE orb_slam3 log)

# 把 compile_commands.json 导出到本文件旁边（固定路径），供智能感知读取。
# AGP 生成在 app/.cxx/<类型>/<哈希>/<abi>/ 下，哈希目录会变，而 compileCommands 不支持通配符。
add_custom_target(export_compile_commands
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
    COMMENT "导出 compile_commands.json")

# 必须挂成依赖：AGP 只构建需要的 .so，不构建 all，只写 ALL 不会执行。
add_dependencies(slam_jni export_compile_commands)
