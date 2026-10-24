// 把 C++ 的 stdout/stderr 接入 logcat（tag=slam）。见 .cpp。
#pragma once

namespace slamlog {

/** 安装重定向并启动读线程，然后打一行日志自证。只应由 JNI_OnLoad 调一次。 */
void install();

}  // namespace slamlog
