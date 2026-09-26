# M1：最小构建——只编桥本身。ORB-SLAM3 / OpenCV 从 M3 起才进来。
cmake_minimum_required(VERSION 3.22.1)
project(slam_jni LANGUAGES CXX)

add_library(slam_jni SHARED jni_bridge.cpp)
target_link_libraries(slam_jni log)   # M2 的 __android_log_write 要用

# 把 compile_commands.json 导出到本文件旁边（固定路径）。
# 智能感知要读它，但 AGP 生成在 app/.cxx/<类型>/<哈希>/<abi>/ 下——
# 哈希目录会变，而 compileCommands 不支持通配符。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(export_compile_commands
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_CURRENT_LIST_DIR}/compile_commands.json
    COMMENT "导出 compile_commands.json")

# ⚠️ 必须挂成依赖，光写 ALL 没用：AGP 只构建 libslam_jni.so，
# **不构建 all**，所以 ALL 形同虚设（实测 build.ninja 里有它，但没跑）。
add_dependencies(slam_jni export_compile_commands)
