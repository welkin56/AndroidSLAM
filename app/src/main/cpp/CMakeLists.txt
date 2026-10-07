# 阶段五 M3：把 ORB-SLAM3 交叉编译成 liborb_slam3.so，和 M1/M2 的 JNI 层一起进 APK。
#
# 依赖分两类，判据和「对 vendored 副本做了哪几处移植改动」都记在 thirdparty/README.md；
# 每处改动的理由见 docs/changes/2026-09-27-m3-cross-compile-orb-slam3.md。
# 一句话：自带的是 ORB-SLAM3 自己 fork 过、版本敏感的小库（DBoW2 / g2o / Sophus），
# 外装的是通用成熟、体积大的（OpenCV / Eigen / Boost）。
cmake_minimum_required(VERSION 3.22.1)
# ⚠️ 这一句必须在任何 target 创建之前：CMAKE_EXPORT_COMPILE_COMMANDS 是在 target 创建时
# 用来初始化它的 EXPORT_COMPILE_COMMANDS 属性的，写在文件末尾就形同没写。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# ⚠️ 必须带 C：g2o 里有一个 os_specific.c，只开 CXX 会在 configure 阶段就报没有 C 编译器。
project(slam_jni LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 必须在任何 target 创建之前。AGP 的 Debug **既不传 -O 也不定义 NDEBUG**，
# 而 PC 那份是 Release（-O3 -DNDEBUG）
#   -O0      真机单帧 600 ms，撞上 ORB-SLAM3 的「时间戳跳变」守卫，反复清图
#   缺 NDEBUG Optimizer.cc:2837 的 assert 会把 App 打崩（PC 上那个 assert 被编掉了）
add_compile_options(-O3 -DNDEBUG)

set(CPP_DIR  ${CMAKE_CURRENT_LIST_DIR})
set(TP_DIR   ${CPP_DIR}/thirdparty)
set(ORB_ROOT ${TP_DIR}/orb_slam3)

# ---- OpenCV ----
# 外装：通用、成熟、体积大（872 MB），官方有预编译发布物，没有理由塞进仓库。
# 默认按本机布局指向 SLAMAPP/OpenCV-android-sdk；换机器用 -DOPENCV_ANDROID_SDK=... 指过去。
# ⚠️ SDK 的内部目录结构一个字都不能动：OpenCVConfig.cmake 靠自己在树里的位置往上数四级
# 算出 SDK 根，挪了它就找不到库。
get_filename_component(_opencv_default ${CPP_DIR}/../../../../../../OpenCV-android-sdk ABSOLUTE)
set(OPENCV_ANDROID_SDK ${_opencv_default} CACHE PATH "OpenCV 官方 Android SDK 的根目录")
set(OpenCV_DIR ${OPENCV_ANDROID_SDK}/sdk/native/jni)
# 它靠 ANDROID_NDK_ABI_NAME 去找 abi-<abi> 子目录，而 NDK 的 toolchain 只设 ANDROID_ABI。
# 不显式指定，它会去找一个不存在的目录并报 ABI 不兼容。
set(OpenCV_CONFIG_SUBDIR /abi-${ANDROID_ABI})
# 只点名要这几个模块。不加限定的话，全量列表里会带上 opencv_java —— 一个 18 MB 的 .so，
# Java 绑定我们完全用不上。
find_package(OpenCV REQUIRED core imgproc imgcodecs features2d calib3d flann)

# ---- Boost ----
# 躲不掉：12 个文件里的 serialize() 没有 #ifdef 保护，哪怕一行地图都不存也要链上。
# 而 Boost 没有官方安卓发布物，得自己编 —— tools/setup-boost-android.sh 一条命令。
set(BOOST_ROOT_ANDROID ${TP_DIR}/boost-android CACHE PATH "安卓版 Boost 的安装前缀")
# ⚠️ 路径写死，别用 find_library：NDK 的 toolchain 把 find_* 的搜索根锁在 sysroot 里
# （CMAKE_FIND_ROOT_PATH_MODE_LIBRARY=ONLY），PATHS 给的目录会被重新挂到 sysroot 下面，
# 结果就是"装在哪都找不到"。setup 脚本用 --layout=system，产物名是确定的。
set(BOOST_SERIALIZATION_LIB ${BOOST_ROOT_ANDROID}/lib/libboost_serialization.a)
if(NOT EXISTS ${BOOST_SERIALIZATION_LIB})
    message(FATAL_ERROR
        "找不到安卓版 Boost.Serialization。先跑 tools/setup-boost-android.sh，"
        "或加 -DBOOST_ROOT_ANDROID=<装在哪>。找的是：${BOOST_SERIALIZATION_LIB}")
endif()

# ---- Eigen ----
# 外装，不自带 —— 和 ORB-SLAM3 在 PC 上的分法一致：Eigen 通用、API 稳定、纯头文件、
# 系统包（apt libeigen3-dev）维护得好，所以用系统那份。它和 g2o/DBoW2/Sophus 不同：
# 后面三个是 ORB-SLAM3 自己 fork 过、版本敏感的，必须跟着代码走。
# ⚠️ 难在"找得到"：NDK 的 toolchain 把 find_* 的搜索根锁在 sysroot 里
# （CMAKE_FIND_ROOT_PATH_MODE_*=ONLY），系统装的 Eigen 会被直接无视。
# NO_CMAKE_FIND_ROOT_PATH 就是绕开那把锁的开关。
find_path(EIGEN3_CONFIG_DIR Eigen3Config.cmake
          PATHS /usr/share/eigen3/cmake /usr/lib/cmake/eigen3 /usr/local/share/eigen3/cmake
          NO_CMAKE_FIND_ROOT_PATH)
if(NOT EIGEN3_CONFIG_DIR)
    message(FATAL_ERROR "找不到系统的 Eigen3。装一下：sudo apt install libeigen3-dev")
endif()
set(Eigen3_DIR ${EIGEN3_CONFIG_DIR} CACHE PATH "系统装的 Eigen3 的 cmake 配置目录")
# 我们自己先找一次：EIGEN3_INCLUDE_DIR 由 Eigen 官方配置算出来（不硬编码路径），
# 而且结果进缓存，g2o 内部那次 FIND_PACKAGE(Eigen3) 直接命中，不会再去 sysroot 里瞎找。
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

# 这两家的 CMakeLists 把产物写死到各自源码树的 lib/ 下（上游留给 PC 版用）。
# 用 target 属性改到构建目录里 —— 属性优先于那个旧的 LIBRARY_OUTPUT_PATH 变量。
set(_slam_out ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
if(NOT _slam_out)
    set(_slam_out ${CMAKE_BINARY_DIR})
endif()
set_target_properties(DBoW2 g2o PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${_slam_out}
    ARCHIVE_OUTPUT_DIRECTORY ${_slam_out})

# ---- ORB-SLAM3 本体（Viewer / MapDrawer 已是空壳）----
#
# ⚠️ `orb_track.cpp` 是**我们自己写的**，却编在这个 target 里。为什么：
# 它要构造 `cv::Mat`，而 `slam_jni` 那个 target **刻意不链 OpenCV** ——
# 进程里已经有一份静态 OpenCV 了（在 orb_slam3 里），再来一份会让 `cv::` 的全局状态
# 变成两套。真机上为此崩过一次（两份 `cv::parallel_for_`，背后各带一套 TBB 全局状态），
# 见 docs/changes/2026-09-28-m5-slam-on-phone.md。**这一层包装就是为了让那条边界成立。**
add_library(orb_slam3 SHARED
    ${CPP_DIR}/md5.cpp                # 顶掉上游的 -lcrypto
    ${CPP_DIR}/orb_track.cpp          # 我们写的；为什么编在这儿见上面那段
    ${CPP_DIR}/measure_center.cpp     # 同上 —— 它碰 `cv::KeyPoint` 和 `MapPoint`
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

# md5.h 只给 orb_slam3 用（System.cc 里那一行 include）。不放进上面的全局
# include_directories —— CMake 的 include 路径粒度只能到目录，没有"只加一个文件"的写法，
# 但 target 级能限到最窄：不会漏给 g2o、DBoW2、slam_jni。
target_include_directories(orb_slam3 PRIVATE ${CPP_DIR})

# ⚠️⚠️ **全部 PRIVATE，尤其是 DBoW2。**
#
# 这条踩过一次，而且**真机崩了**：DBoW2 现在是**静态库**（见它的 CMakeLists），
# 而它 `target_link_libraries(DBoW2 ${OpenCV_LIBS})` 用默认签名（= PUBLIC）链着**静态** OpenCV。
# 用 PUBLIC 链 DBoW2 会把 OpenCV 一路传到 slam_jni —— 于是**同一个进程里出现两份 `cv::`**，
# 其中 `cv::parallel_for_` 背后带着 **TBB 的全局状态**（这份 OpenCV 开了 TBB 后端）：
# `GaussianBlur` 从这库调进去、执行到那库那份实现，状态互相踩 → `SIGSEGV`。
#
# 教训：**「两份逐字节相同的静态库副本」不是体积问题，是正确性问题。**
# DBoW2 静态链进来之后它的代码已经在 liborb_slam3.so 里，slam_jni 不需要再链它。
target_link_libraries(orb_slam3 PRIVATE
    DBoW2 g2o
    ${OpenCV_LIBS}
    ${BOOST_SERIALIZATION_LIB}
    z log)

# ---- JNI 层 ----
# 链上 orb_slam3：M4 就要用到，而且这样 liborb_slam3.so 的静态初始化会在
# App 启动时跑一遍 —— M2 那条「静态初始化期的输出接不接得到」的遗留正好在这里验。
add_library(slam_jni SHARED
    jni_bridge.cpp        # JNI 符号层：翻译 + 转发
    log_redirect.cpp      # 基础设施：把 fd 1/2 接进 logcat
    slam_engine.cpp       # 构造并持有 ORB_SLAM3::System
    )
# ⚠️ **只给头文件路径，绝不链它。**
# `ImuTypes.h` / `System.h` 自己就 include OpenCV，所以编的时候要能找到那些头。
# 但**头文件不产生符号引用** —— 链了才会，而链了进程里就会有两份 `cv::`。
# 这是那条边界的另一半：能在 `slam_jni` 里出现的是**声明**，不是实现。
target_include_directories(slam_jni PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(slam_jni PRIVATE orb_slam3 log)

# 把 compile_commands.json 导出到本文件旁边（固定路径）。
# 智能感知要读它，但 AGP 生成在 app/.cxx/<类型>/<哈希>/<abi>/ 下——
# 哈希目录会变，而 compileCommands 不支持通配符。
# ⚠️ 打开它的那句 set() 在最上面 —— 它是在**创建每个 target 时**用来初始化 target 属性的，
# 写在这里对那些早就建好的 target（DBoW2 / g2o / orb_slam3）无效，整个构建都不会有导出。
add_custom_target(export_compile_commands
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
    COMMENT "导出 compile_commands.json")

# ⚠️ 必须挂成依赖，光写 ALL 没用：AGP 只构建它要的那几个 .so，
# **不构建 all**，所以 ALL 形同虚设（实测 build.ninja 里有它，但没跑）。
add_dependencies(slam_jni export_compile_commands)
