// 把 C++ 世界的 stdout/stderr 接进 logcat（tag=slam）。见 .cpp 顶部的说明。
#pragma once

namespace slamlog {

/** 装重定向 + 起读线程，然后打一行到 logcat 自证。只该由 JNI_OnLoad 调一次。 */
void install();

}  // namespace slamlog
